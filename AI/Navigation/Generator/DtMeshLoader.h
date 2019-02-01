#pragma once

class DtMeshLoader
{
public:
	struct NavMeshSetHeader
	{
		int magic;
		int version;
		int numTiles;
		dtNavMeshParams params;
	};

	struct NavMeshTileHeader
	{
		dtTileRef tileRef;
		int dataSize;
	};

public:
	static void SaveAll(const char* path, const dtNavMesh* mesh);
	static bool LoadAll(const char* path, dtNavMesh*& OutputNavMesh, dtQueryFilter*& OutputFilter);

	static bool LoadNavMesh(const char* path, dtNavMesh*& OutputNavMesh);
	static bool LoadFilter(const char * Path, dtQueryFilter*& OutputFilter);

	static bool LoadTestCase(const char* Path, FVector& StartLoc, FVector& EndLoc, FNavMeshPath & EnginePath);
	static bool LoadTestCase(int Num, FVector& StartLoc, FVector& EndLoc, FNavMeshPath & EnginePath);
};