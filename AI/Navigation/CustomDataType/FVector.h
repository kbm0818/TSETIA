#pragma once

struct FVector
{
public:
	FVector();
	FVector(float x, float y, float z);
	FVector(float x);

	float X;
	float Y;
	float Z;

    FVector operator-(const FVector& V) const
	{
		return FVector(X - V.X, Y - V.Y, Z - V.Z);
	}
	FVector operator+(const FVector& V) const
	{
		return FVector(X + V.X, Y + V.Y, Z + V.Z);
	}

	float Size() const
	{
		return sqrtf(X*X + Y*Y + Z*Z);
	}


	/** A zero vector (0,0,0) */
	static const FVector ZeroVector;

	/** One vector (1,1,1) */
	static const FVector OneVector;

	/** World up vector (0,0,1) */
	static const FVector UpVector;

	/** Unreal forward vector (1,0,0) */
	static const FVector ForwardVector;

	/** Unreal right vector (0,1,0) */
	static const FVector RightVector;

	/** Unreal right vector (0,1,0) */
	static const FVector MaxVector;

	static FVector Recast2UnrealPoint(const float* RecastPoint)
	{
		return FVector(-RecastPoint[0], -RecastPoint[2], RecastPoint[1]);
	}

	static FVector Recast2UnrVector(float const* R)
	{
		return Recast2UnrealPoint(R);
	}

	static FVector Unreal2RecastPoint(const FVector& UnrealPoint)
	{
		return FVector(-UnrealPoint.X, UnrealPoint.Z, -UnrealPoint.Y);
	}

	static FVector Unreal2RecastPoint(const float* UnrealPoint)
	{
		return FVector(-UnrealPoint[0], UnrealPoint[2], -UnrealPoint[1]);
	}
	
	static float Dist(const FVector &V1, const FVector &V2);
	static float DistSquared(const FVector &V1, const FVector &V2);



	bool IsNearlyZero(float Tolerance = KINDA_SMALL_NUMBER);

	__forceinline FVector operator/(float Scale) const
	{
		const float RScale = 1.f / Scale;
		return FVector(X * RScale, Y * RScale, Z * RScale);
	}
	__forceinline bool operator==(FVector target) const
	{
		bool check = true;

		check &= (this->X == target.X);
		check &= (this->Y == target.Y);
		check &= (this->Z == target.Z);
		
		return check;
	}
};