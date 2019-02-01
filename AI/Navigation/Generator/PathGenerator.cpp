#include "stdafx.h"
#include "PathGenerator.h"
#include <windows.h>

#define MAX_COMMON_NODES 2048

////////////////////////////////////////////////////////////////////////////////////
PathGenerator* PathGenerator::instance = NULL;

void PathGenerator::Create()
{
	assert(instance == NULL);

	instance = new PathGenerator();
}

void PathGenerator::Delete()
{
	SAFE_DELETE(instance);
}

PathGenerator * PathGenerator::Get()
{
	assert(instance != NULL);

	return instance;
}

////////////////////////////////////////////////////////////////////////////////////

PathGenerator::PathGenerator()
	: navMeshQuery(nullptr), navMesh(nullptr), filter(nullptr)
{
}

PathGenerator::~PathGenerator()
{
	SAFE_DELETE(filter);
	SAFE_DELETE(navMeshQuery);
	SAFE_DELETE(navMesh);
}

bool PathGenerator::Init(const char * NavigationDataPath)
{
	bool ret = true;
	ret &= DtMeshLoader::LoadAll(NavigationDataPath, navMesh, filter);

	navMeshQuery = dtAllocNavMeshQuery();

	dtStatus result = navMeshQuery->init(navMesh, MAX_COMMON_NODES);
	ret &= dtStatusDetail(result, DT_SUCCESS);
	
	return ret;
}

bool PathGenerator::Generate(FVector & UnrealStartLoc, FVector & UnrealEndLoc, FNavMeshPath & OutputPath)
{
	if (!navMeshQuery || !navMesh)
	{
		printf("Do Init!!!!\n");
		return false;
	}
	OutputPath.Reset();

	FVector RecastStartPos, RecastEndPos;
	NavNodeRef StartPolyID, EndPolyID;
	InitPathfinding(UnrealStartLoc, UnrealEndLoc, *navMeshQuery, filter, RecastStartPos, StartPolyID, RecastEndPos, EndPolyID);

	if (FVector(RecastEndPos - RecastStartPos).IsNearlyZero())
	{
		OutputPath.wantsStringPulling = false;
		OutputPath.wantsPathCorridor = true;
	}

	dtQueryResult PathResult;
	dtStatus FindPathStatus = navMeshQuery->findPath(StartPolyID, EndPolyID, &RecastStartPos.X, &RecastEndPos.X, filter, PathResult, 0);

	if (PathResult.size() == 1 && dtStatusDetail(FindPathStatus, DT_PARTIAL_RESULT))
	{
		FVector RecastHandPlacedPathEnd;
		navMeshQuery->closestPointOnPolyBoundary(StartPolyID, &RecastEndPos.X, &RecastHandPlacedPathEnd.X);

		OutputPath.PathPoints.clear();
		OutputPath.PathPoints.resize(2);
		OutputPath.PathPoints[0] = FNavPathPoint(FVector::Recast2UnrealPoint(&RecastStartPos.X));
		OutputPath.PathPoints[1] = FNavPathPoint(FVector::Recast2UnrealPoint(&RecastHandPlacedPathEnd.X));

		OutputPath.PathCorridor.push_back(PathResult.getRef(0));
		OutputPath.PathCorridorCost.push_back(CalcSegmentCostOnPoly(StartPolyID, filter, RecastHandPlacedPathEnd, RecastStartPos));
	}
	else
	{
		FVector RecastUnrealStartPos, RecastUnrealEndPos;
		RecastUnrealStartPos = FVector::Recast2UnrealPoint(&RecastStartPos.X);
		RecastUnrealEndPos = FVector::Recast2UnrealPoint(&RecastEndPos.X);

		PostProcessPath(FindPathStatus, OutputPath, *navMeshQuery, filter,
			StartPolyID, EndPolyID, RecastUnrealStartPos, RecastUnrealEndPos, RecastStartPos, RecastEndPos,
			PathResult);
	}

	return true;
}

bool PathGenerator::ChangeNavData(const char * NavigationDataPath)
{
	SAFE_DELETE(navMesh);
	SAFE_DELETE(navMeshQuery);

	navMeshQuery = dtAllocNavMeshQuery();
	if (DtMeshLoader::LoadNavMesh(NavigationDataPath, navMesh) == false)
		return false;

	dtStatus result = navMeshQuery->init(navMesh, MAX_COMMON_NODES);
	if (!dtStatusDetail(result, DT_SUCCESS))
		return false;

	return true;
}

bool PathGenerator::ChangeFilter(const char * FilterPath)
{
	SAFE_DELETE(filter);

	bool check = DtMeshLoader::LoadFilter(FilterPath, filter);
	if (check == false || filter == nullptr)
		return false;

	return true;
}

void PathGenerator::ComparePath(FNavMeshPath & FromPath, FNavMeshPath & ToPath)
{
	size_t MaxSize = (FromPath.PathPoints.size() > ToPath.PathPoints.size() ? FromPath.PathPoints.size() : ToPath.PathPoints.size());
	for (size_t i = 0; i < MaxSize; i++)
	{
		PathGenerator::PrintComparePath(FromPath, ToPath, i);
	}
}

void PathGenerator::PrintComparePath(FNavMeshPath & FromPath, FNavMeshPath & ToPath, size_t num)
{
	bool check = true;
	int status = 0;
	if (FromPath.PathPoints.size() <= num)
	{
		status++;
		check = false;
	}

	if (ToPath.PathPoints.size() <= num)
	{
		status++;
		check = false;
	}

	if (status == 0 && !(FromPath.PathPoints[num].Location == ToPath.PathPoints[num].Location))
		check = false;

	if (check == false)
	{
		SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), 4);
	}


	switch (status)
	{
	case 0:
		printf("X:%f Y:%f Z:%f X:%f Y:%f Z:%f\n", FromPath.PathPoints[num].Location.X, FromPath.PathPoints[num].Location.Y, FromPath.PathPoints[num].Location.Z, ToPath.PathPoints[num].Location.X, ToPath.PathPoints[num].Location.Y, ToPath.PathPoints[num].Location.Z);
		break;

	case 1:
		(FromPath.PathPoints.size() > ToPath.PathPoints.size() ? printf("X:%f Y:%f Z:%f\n", FromPath.PathPoints[num].Location.X, FromPath.PathPoints[num].Location.Y, FromPath.PathPoints[num].Location.Z) : printf("\t\t X:%f Y:%f Z:%f\n", ToPath.PathPoints[num].Location.X, ToPath.PathPoints[num].Location.Y, ToPath.PathPoints[num].Location.Z));
		break;

	default:
		break;
	}

	SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), 7);
}

float PathGenerator::CalcSegmentCostOnPoly(NavNodeRef PolyID, const dtQueryFilter * Filter, const FVector & StartLoc, const FVector & EndLoc) const
{
	uint8 AreaID = RECAST_DEFAULT_AREA;
	navMesh->getPolyArea(PolyID, &AreaID);

	const float AreaTravelCost = Filter->getAreaCost(AreaID);
	return AreaTravelCost * (EndLoc - StartLoc).Size();
}

void PathGenerator::PostProcessPath(dtStatus FindPathStatus, FNavMeshPath& Path,
	dtNavMeshQuery& NavQuery, dtQueryFilter* Filter,
	uint64_t StartPolyID, uint64_t EndPolyID,
	FVector& StartLoc, FVector& EndLoc,
	FVector& RecastStartPos, FVector& RecastEndPos,
	dtQueryResult& PathResult
)
{
	// note that for recast partial path is successful, while we treat it as failed, just marking it as partial
	if (!dtStatusSucceed(FindPathStatus))
		return;

	// check if navlink poly at end of path is allowed
	int PathSize = PathResult.size();
	if ((PathSize > 1) && true)
	{
		unsigned short PolyFlags = 0;
		navMesh->getPolyFlags(PathResult.getRef(PathSize - 1), &PolyFlags);

		if (PolyFlags & false) // TODO :: ARecastNavMesh..  Need Navigation Modul
		{
			PathSize--;
		}
	}

	Path.PathCorridorCost.clear();
	Path.PathCorridorCost.resize(PathSize);

	if (PathSize == 1)
	{
		// failsafe cost for single poly corridor
		Path.PathCorridorCost[0] = CalcSegmentCostOnPoly(StartPolyID, Filter, EndLoc, StartLoc);
	}
	else
	{
		for (int i = 0; i < PathSize; i++)
		{
			Path.PathCorridorCost[i] = PathResult.getCost(i);
		}
	}

	// copy over corridor poly data
	Path.PathCorridor.clear();
	Path.PathCorridor.resize(PathSize);

	uint64_t* DestCorridorPoly = &(Path.PathCorridor[0]);
	for (int i = 0; i < PathSize; ++i, ++DestCorridorPoly)
	{
		*DestCorridorPoly = PathResult.getRef(i);
	}

	//Path.OnPathCorridorUpdated();

#if STATS
	if (dtStatusDetail(FindPathStatus, DT_OUT_OF_NODES))
	{
		INC_DWORD_STAT(STAT_Navigation_OutOfNodesPath);
	}

	if (dtStatusDetail(FindPathStatus, DT_PARTIAL_RESULT))
	{
		INC_DWORD_STAT(STAT_Navigation_PartialPath);
	}
#endif

	//if (Path.WantsStringPulling())
	if (Path.wantsStringPulling)
	{
		FVector UseEndLoc = EndLoc;

		// if path is partial (path corridor doesn't contain EndPolyID), find new RecastEndPos on last poly in corridor
		if (dtStatusDetail(FindPathStatus, DT_PARTIAL_RESULT))
		{
			uint64_t LastPolyID = Path.PathCorridor[Path.PathCorridor.size() - 1];
			float NewEndPoint[3];

			const dtStatus NewEndPointStatus = NavQuery.closestPointOnPoly(LastPolyID, &RecastEndPos.X, NewEndPoint);
			if (dtStatusSucceed(NewEndPointStatus))
			{
				UseEndLoc = FVector::Recast2UnrealPoint(NewEndPoint);
			}
		}
		PerformStringPulling(StartLoc, UseEndLoc, Path);
	}
	else
	{
		Path.PathPoints.resize(2);
		Path.PathPoints[0] = FNavPathPoint(StartLoc, StartPolyID);
		Path.PathPoints[1] = FNavPathPoint(EndLoc, EndPolyID);

		// collect all custom links Ids
		for (size_t Idx = 0; Idx < Path.PathCorridor.size(); Idx++)
		{
			const dtOffMeshConnection* OffMeshCon = navMesh->getOffMeshConnectionByRef(Path.PathCorridor[Idx]);
			if (OffMeshCon)
			{
				Path.CustomLinkIds.push_back(OffMeshCon->userId);
			}
		}
	}

	if (Path.wantsPathCorridor)
	{
		std::vector<FNavigationPortalEdge> PathCorridorEdges;
		GetEdgesForPathCorridorImpl(&Path.PathCorridor, &PathCorridorEdges, NavQuery);
		//Path.SetPathCorridorEdges(PathCorridorEdges);*/
	}
}

bool PathGenerator::InitPathfinding(const FVector & UnrealStart, const FVector & UnrealEnd, const dtNavMeshQuery & Query, const dtQueryFilter * Filter, FVector & RecastStart, dtPolyRef & StartPoly, FVector & RecastEnd, dtPolyRef & EndPoly) const
{
	const float Extent[3] = { DEFAULT_NAV_QUERY_EXTENT_HORIZONTAL, DEFAULT_NAV_QUERY_EXTENT_VERTICAL, DEFAULT_NAV_QUERY_EXTENT_HORIZONTAL  };

	const FVector RecastStartToProject = FVector::Unreal2RecastPoint(UnrealStart);
	const FVector RecastEndToProject = FVector::Unreal2RecastPoint(UnrealEnd);

	StartPoly = INVALID_NAVNODEREF;
	Query.findNearestPoly(&RecastStartToProject.X, Extent, Filter, &StartPoly, &RecastStart.X);
	if (StartPoly == INVALID_NAVNODEREF)
	{
		printf("FPImplRecastNavMesh::InitPathfinding start point not on navmesh");
		return false;
	}

	EndPoly = INVALID_NAVNODEREF;
	Query.findNearestPoly(&RecastEndToProject.X, Extent, Filter, &EndPoly, &RecastEnd.X);
	if (EndPoly == INVALID_NAVNODEREF)
	{
		printf("FPImplRecastNavMesh::InitPathfinding end point not on navmesh");
		return false;
	}

	return true;
}

void PathGenerator::PerformStringPulling(FVector & StartLoc, FVector & EndLoc, FNavMeshPath& Path)
{
	if (Path.PathCorridor.size())
	{
		Path.bStringPulled = FindStraightPath(StartLoc, EndLoc, Path);
		//, PathCorridor, PathPoints, &CustomLinkIds
	}
}

bool PathGenerator::FindStraightPath(FVector & StartLoc, FVector & EndLoc, FNavMeshPath& Path)
{
	const FVector RecastStartPos = FVector::Unreal2RecastPoint(StartLoc);
	const FVector RecastEndPos = FVector::Unreal2RecastPoint(EndLoc);
	bool bResult = false;

	dtQueryResult StringPullResult;
	const dtStatus StringPullStatus = navMeshQuery->findStraightPath(&RecastStartPos.X, &RecastEndPos.X,
		&Path.PathCorridor[0], (const int)Path.PathCorridor.size(), StringPullResult, DT_STRAIGHTPATH_AREA_CROSSINGS);

	Path.PathPoints.clear();

	if (dtStatusSucceed(StringPullStatus))
	{
		size_t pathPointsSize = Path.PathPoints.size();
		Path.PathPoints.resize(pathPointsSize + StringPullResult.size());
		memset(&Path.PathPoints[pathPointsSize], 0, StringPullResult.size());

		FNavPathPoint* CurVert = &Path.PathPoints[0];

		for (int32 VertIdx = 0; VertIdx < StringPullResult.size(); ++VertIdx)
		{
			const float* CurRecastVert = StringPullResult.getPos(VertIdx);
			CurVert->Location = FVector::Recast2UnrVector(CurRecastVert);
			CurVert->NodeRef = StringPullResult.getRef(VertIdx);

			FNavMeshNodeFlags CurNodeFlags(0);
			CurNodeFlags.PathFlags = StringPullResult.getFlag(VertIdx);

			uint8 AreaID = RECAST_DEFAULT_AREA;
			navMesh->getPolyArea(CurVert->NodeRef, &AreaID);
			CurNodeFlags.Area = AreaID;

			/*const UClass* AreaClass = NavMeshOwner->GetAreaClass(AreaID);
			const UNavArea* DefArea = AreaClass ? ((UClass*)AreaClass)->GetDefaultObject<UNavArea>() : NULL;
			CurNodeFlags.AreaFlags = DefArea ? DefArea->GetAreaFlags() : 0;

			CurVert->Flags = CurNodeFlags.Pack();*/

			CurVert->Flags = 0; // TODO:: For Test

			if ((CurNodeFlags.PathFlags & DT_STRAIGHTPATH_OFFMESH_CONNECTION))
			{
				const dtOffMeshConnection* OffMeshCon = navMesh->getOffMeshConnectionByRef(CurVert->NodeRef);
				if (OffMeshCon)
				{
					CurVert->CustomLinkId = OffMeshCon->userId;
					Path.CustomLinkIds.push_back(OffMeshCon->userId);
				}
			}

			CurVert++;
		}

		Path.PathPoints[Path.PathPoints.size() - 1].NodeRef = Path.PathCorridor[Path.PathCorridor.size() - 1];
		bResult = true;
	}

	return bResult;
}

void PathGenerator::GetEdgesForPathCorridorImpl(const std::vector<NavNodeRef>* PathCorridor, std::vector<FNavigationPortalEdge>* PathCorridorEdges, const dtNavMeshQuery & NavQuery)
{
	const int32 CorridorLenght = (int32)PathCorridor->size();

	PathCorridorEdges->clear();
	PathCorridorEdges->resize(CorridorLenght - 1);
	for (int32 i = 0; i < CorridorLenght - 1; ++i)
	{
		unsigned char FromType = 0, ToType = 0;
		float Left[3] = { 0.f }, Right[3] = { 0.f };

		NavQuery.getPortalPoints((*PathCorridor)[i], (*PathCorridor)[i + 1], Left, Right, FromType, ToType);

		PathCorridorEdges->push_back(FNavigationPortalEdge(FVector::Recast2UnrVector(Left), FVector::Recast2UnrVector(Right), (*PathCorridor)[i + 1]));
	}
}

DLL bool C_Init(const char* NavigationDataPath)
{
	PathGenerator::Create();
	return PathGenerator::Get()->Init(NavigationDataPath);
}

//
//DLL void C_Generator(FVector UnrealStartLoc, FVector UnrealEndLoc, FVector* OutPath, size_t* PathSize)
//{
//	FNavMeshPath Path;
//	PathGenerator::Get()->Generate(UnrealStartLoc, UnrealEndLoc, Path);
//
//	OutPath = (FVector*)malloc(sizeof(FVector) * (int)Path.PathPoints.size());
//	for (size_t i = 0; i < Path.PathPoints.size(); i++)
//	{
//		OutPath[i] = FVector(Path.PathPoints[i].Location.X, Path.PathPoints[i].Location.Y, Path.PathPoints[i].Location.Z);
//	}
//}
