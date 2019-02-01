#include "stdafx.h"
#include "FVector.h"
#include <math.h>

#define FLT_MAX          3.402823466e+38F

const FVector FVector::ZeroVector(0.0f, 0.0f, 0.0f);
const FVector FVector::OneVector(1.0f, 1.0f, 1.0f);
const FVector FVector::UpVector(0.0f, 0.0f, 1.0f);
const FVector FVector::ForwardVector(1.0f, 0.0f, 0.0f);
const FVector FVector::RightVector(0.0f, 1.0f, 0.0f);
const FVector FVector::MaxVector(FLT_MAX, FLT_MAX, FLT_MAX);

FVector::FVector()
	: X(0), Y(0), Z(0)
{
}

FVector::FVector(float x, float y, float z)
	: X(x), Y(y), Z(z)
{
}

FVector::FVector(float x)
	:X(x), Y(x), Z(x)
{
}

float FVector::Dist(const FVector & V1, const FVector & V2)
{
	return sqrt(FVector::DistSquared(V1, V2));
}

float FVector::DistSquared(const FVector & V1, const FVector & V2)
{
	return (V2.X - V1.X) * (V2.X - V1.X) + (V2.Y - V1.Y) * (V2.Y - V1.Y) + (V2.Z - V1.Z) * (V2.Z - V1.Z);
}

bool FVector::IsNearlyZero(float Tolerance)
{
	return
		abs(X) <= Tolerance
		&& abs(Y) <= Tolerance
		&& abs(Z) <= Tolerance;
}
