// Copyright 1998-2018 Epic Games, Inc. All Rights Reserved.

#pragma once


#include "Navmesh/Detour/DetourNavMeshQuery.h"

struct dtSharedBoundaryEdge
{
	float v0[3];
	float v1[3];
	dtPolyRef p0;
	dtPolyRef p1;
};

struct dtSharedBoundaryData
{
	float Center[3];
	float Radius;
	float AccessTime;
	dtQueryFilter* Filter;
	unsigned char SingleAreaId;
	std::vector<dtSharedBoundaryEdge> Edges;
	//TArray<dtSharedBoundaryEdge> Edges;
	std::vector<dtPolyRef> Polys;

	dtSharedBoundaryData() : Filter(nullptr) {}
};

class dtSharedBoundary
{
public:
	std::vector<dtSharedBoundaryData> Data;
	dtQueryFilter SingleAreaFilter;
	float CurrentTime;
	float NextClearTime;

	void Initialize();
	void Tick(float DeltaTime);

	int FindData(float* Center, float Radius, dtPolyRef ReqPoly, dtQueryFilter* NavFilter) const;
	int FindData(float* Center, float Radius, dtPolyRef ReqPoly, unsigned char SingleAreaId) const;

	int CacheData(float* Center, float Radius, dtPolyRef CenterPoly, dtNavMeshQuery* NavQuery, dtQueryFilter* NavFilter);
	int CacheData(float* Center, float Radius, dtPolyRef CenterPoly, dtNavMeshQuery* NavQuery, unsigned char SingleAreaId);

	void FindEdges(dtSharedBoundaryData& Data, dtPolyRef CenterPoly, dtNavMeshQuery* NavQuery, dtQueryFilter* NavFilter);
	bool HasSample(int Idx) const;

private:

	bool IsValid(int Idx, dtNavMeshQuery* NavQuery, dtQueryFilter* NavFilter) const;
};
