#include "stdafx.h"
#include "FNavMeshPath.h"

std::vector<FNavigationPortalEdge>& FNavMeshPath::GetPathCorridorEdges()
{
	const int32 CorridorLength = PathCorridor.size();
	if (CorridorLength != 0)
	{
	/*	mesh->GetEdgesForPathCorridor(&PathCorridor, &PathCorridorEdges);
		bCorridorEdgesGenerated = PathCorridorEdges.Num() > 0;*/
	}
	return PathCorridorEdges;
}

void FNavMeshPath::Reset()
{
	PathCorridorCost.clear();
	PathCorridor.clear();
	PathPoints.clear();
	CustomLinkIds.clear();

	wantsStringPulling = true;
	wantsPathCorridor = false;
}
