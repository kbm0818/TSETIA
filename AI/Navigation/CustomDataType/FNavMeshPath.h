#pragma once

#define INVALID_NAVNODEREF (0)
#define INVALID_NAVQUERYID int(0)
#define INVALID_NAVDATA unsigned int(0)
#define INVALID_NAVEXTENT (FVector::ZeroVector)

struct FNavigationPortalEdge
{
	FVector Left;
	FVector Right;
	NavNodeRef ToRef;

	FNavigationPortalEdge() : Left(0.f), Right(0.f)
	{}

	FNavigationPortalEdge(const FVector& InLeft, const FVector& InRight, NavNodeRef InToRef)
		: Left(InLeft), Right(InRight), ToRef(InToRef)
	{}

	__forceinline FVector GetPoint(const int32 Index) const
	{
		if(Index >= 0 && Index < 2)
			return ((FVector*)&Left)[Index];
		else
			return FVector();
	}

	__forceinline float GetLength() const { return FVector::Dist(Left, Right); }

	__forceinline FVector GetMiddlePoint() const { return Left + (Right - Left) / 2; }

};

struct FNavLocation
{
	/** location relative to path's base */
	FVector Location;

	/** node reference in navigation data */
	unsigned long long NodeRef;

	FNavLocation() : Location(FVector::ZeroVector), NodeRef(INVALID_NAVNODEREF) {}
	explicit FNavLocation(const FVector& InLocation, uint64_t InNodeRef = INVALID_NAVNODEREF)
		: Location(InLocation), NodeRef(InNodeRef) {}

	/** checks if location has associated navigation node ref */
	__forceinline bool HasNodeRef() const { return NodeRef != INVALID_NAVNODEREF; }

	__forceinline operator FVector() const { return Location; }
};

struct FNavPathPoint : public FNavLocation
{
	/** extra node flags */
	unsigned int Flags;

	/** unique Id of custom navigation link starting at this point */
	unsigned int CustomLinkId;

	FNavPathPoint() : Flags(0), CustomLinkId(0) {}
	FNavPathPoint(const FVector& InLocation, uint64_t InNodeRef = INVALID_NAVNODEREF, unsigned int InFlags = 0)
		: FNavLocation(InLocation, InNodeRef), Flags(InFlags), CustomLinkId(0) {}

	__forceinline bool operator==(FNavPathPoint target) const
	{
		bool check = true;

		check &= (this->Location.X == target.Location.X);
		check &= (this->Location.Y == target.Location.Y);
		check &= (this->Location.Z == target.Location.Z);

		return check;
	}
};

struct FNavMeshPath
{
	std::vector<float> PathCorridorCost;
	std::vector<uint64_t> PathCorridor;
	std::vector<FNavPathPoint> PathPoints;
	std::vector<unsigned int> CustomLinkIds;
	std::vector<FNavigationPortalEdge> PathCorridorEdges;
	unsigned int bStringPulled : 1;
	bool wantsStringPulling = true;
	bool wantsPathCorridor = false;

	std::vector<FNavigationPortalEdge>& GetPathCorridorEdges();
	//__forceinline void SetPathCorridorEdges(const std::vector<FNavigationPortalEdge>& InPathCorridorEdges) { PathCorridorEdges = InPathCorridorEdges; bCorridorEdgesGenerated = true; }

	void Reset();
};

#define RECAST_STRAIGHTPATH_OFFMESH_CONNECTION 0x04

struct FNavMeshNodeFlags
{
	uint8 PathFlags;
	uint8 Area;
	uint16 AreaFlags;

	FNavMeshNodeFlags() : PathFlags(0), Area(0), AreaFlags(0) {}
	FNavMeshNodeFlags(unsigned int Flags) : PathFlags(Flags), Area(Flags >> 8), AreaFlags(Flags >> 16) {}
	unsigned int Pack() const { return PathFlags | ((unsigned int)Area << 8) | ((unsigned int)AreaFlags << 16); }
	bool IsNavLink() const { return (PathFlags & RECAST_STRAIGHTPATH_OFFMESH_CONNECTION) != 0; }

	FNavMeshNodeFlags& AddAreaFlags(const uint16 InAreaFlags)
	{
		AreaFlags = (AreaFlags | InAreaFlags);
		return *this;
	}
};