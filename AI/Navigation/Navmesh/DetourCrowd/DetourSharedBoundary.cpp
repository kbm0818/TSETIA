// Copyright 1998-2018 Epic Games, Inc. All Rights Reserved.

#include "stdafx.h"
#include "DetourSharedBoundary.h"

void dtSharedBoundary::Initialize()
{
	CurrentTime = 0.0f;
	NextClearTime = 0.0f;

	for (int Idx = 0; Idx < DT_MAX_AREAS; Idx++)
	{
		SingleAreaFilter.setAreaCost(Idx, DT_UNWALKABLE_POLY_COST);
	}
}

void dtSharedBoundary::Tick(float DeltaTime)
{
	CurrentTime += DeltaTime;
	
	// clear unused entries
	if (CurrentTime > NextClearTime)
	{
		const float MaxLifeTime = 2.0f;
		NextClearTime = CurrentTime + MaxLifeTime;

		for (auto It = Data.begin(); It < Data.end(); ++It)
		{
			const float LastAccess = CurrentTime - It->AccessTime;
			if (LastAccess >= MaxLifeTime)
			{
				Data.erase(It);
				//It.RemoveCurrent();
			}
		}
	}
}

int dtSharedBoundary::CacheData(float* Center, float Radius, dtPolyRef CenterPoly, dtNavMeshQuery* NavQuery, dtQueryFilter* NavFilter)
{
	// bail if requested poly is not valid (e.g. rebuild in progress)
	if (NavQuery && !NavQuery->isValidPolyRef(CenterPoly, NavFilter))
	{
		return -1;
	}

	Radius *= 1.5f;

	int DataIdx = FindData(Center, Radius, CenterPoly, NavFilter);
	const bool bHasValidData = IsValid(DataIdx, NavQuery, NavFilter);
	if (!bHasValidData)
	{
		if (DataIdx >= 0)
		{
			// remove in next cleanup
			Data[DataIdx].AccessTime = 0.0f;
		}

		dtSharedBoundaryData NewData;
		dtVcopy(NewData.Center, Center);
		NewData.Radius = Radius;
		NewData.Filter = NavFilter;
		NewData.SingleAreaId = 0;

		FindEdges(NewData, CenterPoly, NavQuery, NavFilter);
		Data.push_back(NewData);
		DataIdx = Data.size() - 1;
	}

	Data[DataIdx].AccessTime = CurrentTime;
	return DataIdx;
}

int dtSharedBoundary::CacheData(float* Center, float Radius, dtPolyRef CenterPoly, dtNavMeshQuery* NavQuery, unsigned char SingleAreaId)
{
	SingleAreaFilter.setAreaCost(SingleAreaId, 1.0f);

	// bail if requested poly is not valid (e.g. rebuild in progress)
	if (NavQuery && !NavQuery->isValidPolyRef(CenterPoly, &SingleAreaFilter))
	{
		return -1;
	}

	Radius *= 1.5f;

	int DataIdx = FindData(Center, Radius, CenterPoly, SingleAreaId);
	const bool bHasValidData = IsValid(DataIdx, NavQuery, &SingleAreaFilter);
	if (!bHasValidData)
	{
		if (DataIdx >= 0)
		{
			// remove in next cleanup
			Data[DataIdx].AccessTime = 0.0f;
		}

		dtSharedBoundaryData NewData;
		dtVcopy(NewData.Center, Center);
		NewData.Radius = Radius;
		NewData.SingleAreaId = SingleAreaId;

		FindEdges(NewData, CenterPoly, NavQuery, &SingleAreaFilter);
		Data.push_back(NewData);
		DataIdx = Data.size() - 1;
		//DataIdx = Data.Add(NewData);
	}

	SingleAreaFilter.setAreaCost(SingleAreaId, DT_UNWALKABLE_POLY_COST);
	Data[DataIdx].AccessTime = CurrentTime;
	return DataIdx;
}

void dtSharedBoundary::FindEdges(dtSharedBoundaryData& SharedData, dtPolyRef CenterPoly, dtNavMeshQuery* NavQuery, dtQueryFilter* NavFilter)
{
	const int MaxWalls = 64;
	int NumWalls = 0;
	float WallSegments[MaxWalls * 3 * 2] = { 0 };
	dtPolyRef WallPolys[MaxWalls * 2] = { 0 };

	const int MaxNeis = 64;
	int NumNeis = 0;
	dtPolyRef NeiPolys[MaxNeis] = { 0 };

	NavQuery->findWallsInNeighbourhood(CenterPoly, SharedData.Center, SharedData.Radius, NavFilter,
		NeiPolys, &NumNeis, MaxNeis, WallSegments, WallPolys, &NumWalls, MaxWalls);
	
	dtSharedBoundaryEdge NewEdge;
	for (int Idx = 0; Idx < NumWalls; Idx++)
	{
		dtVcopy(NewEdge.v0, &WallSegments[Idx * 6]);
		dtVcopy(NewEdge.v1, &WallSegments[Idx * 6 + 3]);
		NewEdge.p0 = WallPolys[Idx * 2];
		NewEdge.p1 = WallPolys[Idx * 2 + 1];

		SharedData.Edges.push_back(NewEdge);

	}

	SharedData.Polys.resize(NumNeis);
	for (int Idx = 0; Idx < NumNeis; Idx++)
	{
		SharedData.Polys.push_back(NeiPolys[Idx]);
	}
}

int dtSharedBoundary::FindData(float* Center, float Radius, dtPolyRef ReqPoly, dtQueryFilter* NavFilter) const
{
	const float RadiusThr = 50.0f;
	const float DistThrSq = (Radius * 0.5f)*(Radius * 0.5f);

	for (size_t Idx = 0; Idx < Data.size(); Idx++)
	{
		if (Data[Idx].Filter == NavFilter)
		{
			const float DistSq = dtVdistSqr(Center, Data[Idx].Center);
			if (DistSq <= DistThrSq && dtAbs(Data[Idx].Radius - Radius) < RadiusThr)
			{
				auto It = std::find(Data[Idx].Polys.begin(), Data[Idx].Polys.end(), ReqPoly);
				if(It != Data[Idx].Polys.end())
				{
					return Idx;
				}
			}
		}
	}

	return -1;
}

int dtSharedBoundary::FindData(float* Center, float Radius, dtPolyRef ReqPoly, unsigned char SingleAreaId) const
{
	const float DistThrSq = (Radius * 0.5f) * (Radius * 0.5f);
	const float RadiusThr = 50.0f;

	for (size_t Idx = 0; Idx < Data.size(); Idx++)
	{
		if (Data[Idx].SingleAreaId == SingleAreaId)
		{
			const float DistSq = dtVdistSqr(Center, Data[Idx].Center);
			if (DistSq <= DistThrSq && dtAbs(Data[Idx].Radius - Radius) < RadiusThr)
			{
				auto It = std::find(Data[Idx].Polys.begin(), Data[Idx].Polys.end(), ReqPoly);
				if (It != Data[Idx].Polys.end())
				{
					return Idx;
				}
			}
		}
	}

	return -1;
}

bool dtSharedBoundary::HasSample(int Idx) const
{
	bool bAllocated = true;
	return (Idx >= 0) &&
		(Idx < (int)Data.size()) &&
		bAllocated;
}

bool dtSharedBoundary::IsValid(int Idx, dtNavMeshQuery* NavQuery, dtQueryFilter* NavFilter) const
{
	bool bValid = HasSample(Idx);
	if (bValid)
	{
		for (auto It = Data[Idx].Polys.begin(); It < Data[Idx].Polys.end(); ++It)
		{
			const dtPolyRef TestRef = *It;
			const bool bValidRef = NavQuery->isValidPolyRef(TestRef, NavFilter);
			if (!bValidRef)
			{
				bValid = false;
				break;
			}
		}
	}

	return bValid;
}
