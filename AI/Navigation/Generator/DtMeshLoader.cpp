#include "stdafx.h"
#include "DtMeshLoader.h"
#include <string>

static const int NAVMESHSET_MAGIC = 'M' << 24 | 'S' << 16 | 'E' << 8 | 'T'; //'MSET';
static const int NAVMESHSET_VERSION = 1;

void DtMeshLoader::SaveAll(const char * path, const dtNavMesh * mesh)
{
	if (!mesh) return;

	FILE* fp = nullptr;
	fopen_s(&fp, path, "wb");
	if (!fp)
		return;

	// Store header.
	NavMeshSetHeader header;
	header.magic = NAVMESHSET_MAGIC;
	header.version = NAVMESHSET_VERSION;
	header.numTiles = 0;
	for (int i = 0; i < mesh->getMaxTiles(); ++i)
	{
		const dtMeshTile* tile = mesh->getTile(i);
		if (!tile || !tile->header || !tile->dataSize) continue;
		header.numTiles++;
	}
	memcpy(&header.params, mesh->getParams(), sizeof(dtNavMeshParams));
	fwrite(&header, sizeof(NavMeshSetHeader), 1, fp);

	// Store tiles.
	for (int i = 0; i < mesh->getMaxTiles(); ++i)
	{
		const dtMeshTile* tile = mesh->getTile(i);
		if (!tile || !tile->header || !tile->dataSize) continue;

		NavMeshTileHeader tileHeader;
		tileHeader.tileRef = mesh->getTileRef(tile);
		tileHeader.dataSize = tile->dataSize;
		fwrite(&tileHeader, sizeof(tileHeader), 1, fp);

		fwrite(tile->data, tile->dataSize, 1, fp);
	}

	fclose(fp);
}

bool DtMeshLoader::LoadAll(const char* path, dtNavMesh*& OutputNavMesh, dtQueryFilter*& OutputFilter)
{
	SAFE_DELETE(OutputNavMesh);
	SAFE_DELETE(OutputFilter);

	FILE* fp = nullptr;
	fopen_s(&fp, path, "rb");
	if (!fp)
		return false;

	// Read header.
	NavMeshSetHeader header;
	size_t readLen = fread(&header, sizeof(NavMeshSetHeader), 1, fp);
	if (header.magic != NAVMESHSET_MAGIC)
	{
		fclose(fp);
		return false;
	}
	if (header.version != NAVMESHSET_VERSION)
	{
		fclose(fp);
		return false;
	}

	OutputNavMesh = dtAllocNavMesh();
	if (!OutputNavMesh)
	{
		fclose(fp);
		return false;
	}
	dtStatus status = OutputNavMesh->init(&header.params);
	if (dtStatusFailed(status))
	{
		fclose(fp);
		return false;
	}

	// Read tiles.
	for (int i = 0; i < header.numTiles; ++i)
	{
		NavMeshTileHeader tileHeader;
		fread(&tileHeader, sizeof(tileHeader), 1, fp);
		if (!tileHeader.tileRef || !tileHeader.dataSize)
			return false;

		unsigned char* data = (unsigned char*)dtAlloc(tileHeader.dataSize, DT_ALLOC_PERM);
		if (!data)
			return false;

		memset(data, 0, tileHeader.dataSize);
		fread(data, tileHeader.dataSize, 1, fp);

		OutputNavMesh->addTile(data, tileHeader.dataSize, DT_TILE_FREE_DATA, tileHeader.tileRef, 0);
	}

	bool temp;
	readLen = fread(&temp, sizeof(bool), 1, fp);
	if (readLen != 1)
		return false;

	OutputFilter = new dtQueryFilter(temp);
	if (OutputFilter == nullptr)
		return false;

	fread(&OutputFilter->data, sizeof(dtQueryFilterData), 1, fp);
	if (readLen != 1)
		return false;

	fclose(fp);

	return true;
}

bool DtMeshLoader::LoadNavMesh(const char * path, dtNavMesh *& OutputNavMesh)
{
	SAFE_DELETE(OutputNavMesh);

	FILE* fp = nullptr;
	fopen_s(&fp, path, "rb");
	if (!fp)
		return false;

	// Read header.
	NavMeshSetHeader header;
	size_t readLen = fread(&header, sizeof(NavMeshSetHeader), 1, fp);
	if (header.magic != NAVMESHSET_MAGIC)
	{
		fclose(fp);
		return false;
	}
	if (header.version != NAVMESHSET_VERSION)
	{
		fclose(fp);
		return false;
	}

	OutputNavMesh = dtAllocNavMesh();
	if (!OutputNavMesh)
	{
		fclose(fp);
		return false;
	}
	dtStatus status = OutputNavMesh->init(&header.params);
	if (dtStatusFailed(status))
	{
		fclose(fp);
		return false;
	}

	// Read tiles.
	for (int i = 0; i < header.numTiles; ++i)
	{
		NavMeshTileHeader tileHeader;
		fread(&tileHeader, sizeof(tileHeader), 1, fp);
		if (!tileHeader.tileRef || !tileHeader.dataSize)
			return false;

		unsigned char* data = (unsigned char*)dtAlloc(tileHeader.dataSize, DT_ALLOC_PERM);
		if (!data)
			return false;

		memset(data, 0, tileHeader.dataSize);
		fread(data, tileHeader.dataSize, 1, fp);

		OutputNavMesh->addTile(data, tileHeader.dataSize, DT_TILE_FREE_DATA, tileHeader.tileRef, 0);
	}

	fclose(fp);

	return true;
}

bool DtMeshLoader::LoadFilter(const char * Path, dtQueryFilter*& OutputFilter)
{
	SAFE_DELETE(OutputFilter);

	FILE* fp = nullptr;
	fopen_s(&fp, Path, "rb");
	if (!fp)
		return false;

	bool temp;
	size_t readLen = fread(&temp, sizeof(bool), 1, fp);
	if (readLen != 1)
		return false;

	OutputFilter = new dtQueryFilter(temp);
	if (OutputFilter == nullptr)
		return false;

	fread(&OutputFilter->data, sizeof(dtQueryFilterData), 1, fp);
	if (readLen != 1)
		return false;
	
	return true;
}

bool DtMeshLoader::LoadTestCase(const char * Path, FVector & StartLoc, FVector & EndLoc, FNavMeshPath & EnginePath)
{
	FILE* fp = nullptr;
	fopen_s(&fp, Path, "rb");
	if (!fp)
		return false;

	size_t readLen = fread(&StartLoc, sizeof(float), 3, fp);
	if (readLen != 3)
	{
		return false;
	}

	readLen = fread(&EndLoc, sizeof(float), 3, fp);
	if (readLen != 3)
	{
		return false;
	}

	int dataSize = 0;
	readLen = fread(&dataSize, sizeof(int), 1, fp);
	if (readLen != 1)
	{
		return false;
	}

	EnginePath.PathPoints.resize(dataSize);
	for (int i = 0; i < dataSize; i++)
	{
		FVector tempVector;
		readLen = fread(&tempVector, sizeof(float), 3, fp);
		if (readLen != 3)
		{
			return false;
		}
		EnginePath.PathPoints[i].Location = tempVector;
	}

	fclose(fp);

	return true;
}

using namespace std;
bool DtMeshLoader::LoadTestCase(int Num, FVector & StartLoc, FVector & EndLoc, FNavMeshPath & EnginePath)
{
	string path = "E://testCase";
	string end = ".bin";
	string num = std::to_string(Num);

	return DtMeshLoader::LoadTestCase(string(path + num + end).c_str(), StartLoc, EndLoc, EnginePath);
}
