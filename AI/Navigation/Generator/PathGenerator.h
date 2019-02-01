#pragma once

class PathGenerator
{
public:
	PathGenerator();
	~PathGenerator();

	static void Create();
	static void Delete();

	static PathGenerator* Get();

	bool Init(const char* NavigationDataPath);
	bool Generate(FVector& UnrealStartLoc, FVector& UnrealEndLoc, FNavMeshPath& OutputPath);

	bool ChangeNavData(const char* NavigationDataPath);
	bool ChangeFilter(const char* FilterPath);

	static void ComparePath(FNavMeshPath& FromPath, FNavMeshPath& ToPath);

private:
	static void PrintComparePath(FNavMeshPath& FromPath, FNavMeshPath& ToPath, size_t num);

	float CalcSegmentCostOnPoly(NavNodeRef PolyID, const dtQueryFilter* Filter, const FVector& StartLoc, const FVector& EndLoc) const;

	/** Marks path flags, perform string pulling if needed */
	void PostProcessPath(dtStatus PathfindResult, FNavMeshPath& Path,
		dtNavMeshQuery& Query, dtQueryFilter* Filter,
		uint64_t StartNode, uint64_t EndNode,
		FVector& UnrealStart, FVector& UnrealEnd,
		FVector& RecastStart, FVector& RecastEnd,
		dtQueryResult& PathResult);

	bool InitPathfinding(const FVector& UnrealStart, const FVector& UnrealEnd,
		const dtNavMeshQuery& Query, const dtQueryFilter* Filter,
		FVector& RecastStart, dtPolyRef& StartPoly,
		FVector& RecastEnd, dtPolyRef& EndPoly) const;

	void PerformStringPulling(FVector& StartLoc, FVector& EndLoc, FNavMeshPath& Path);
	bool FindStraightPath(FVector & StartLoc, FVector & EndLoc, FNavMeshPath& Path);
	void GetEdgesForPathCorridorImpl(const std::vector<NavNodeRef>* PathCorridor, std::vector<FNavigationPortalEdge>* PathCorridorEdges, const dtNavMeshQuery& NavQuery);

private:
	static PathGenerator* instance;

	dtNavMeshQuery* navMeshQuery;
	dtNavMesh* navMesh;
	dtQueryFilter* filter;

};