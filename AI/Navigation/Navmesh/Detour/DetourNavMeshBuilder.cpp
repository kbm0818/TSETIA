// Copyright 1998-2018 Epic Games, Inc. All Rights Reserved.
// Modified version of Recast/Detour's source file

//
// Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
//
// This software is provided 'as-is', without any express or implied
// warranty.  In no event will the authors be held liable for any damages
// arising from the use of this software.
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
// 1. The origin of this software must not be misrepresented; you must not
//    claim that you wrote the original software. If you use this software
//    in a product, an acknowledgment in the product documentation would be
//    appreciated but is not required.
// 2. Altered source versions must be plainly marked as such, and must not be
//    misrepresented as being the original software.
// 3. This notice may not be removed or altered from any source distribution.
//
#include "stdafx.h"
#include "Navmesh/Detour/DetourNavMeshBuilder.h"
#include "Navmesh/Detour/DetourNavMesh.h"
#include "Navmesh/Detour/DetourCommon.h"

static unsigned short MESH_NULL_IDX = 0xffff;


struct BVItem
{
	unsigned short bmin[3];
	unsigned short bmax[3];
	int i;
};

static int compareItemX(const void* va, const void* vb)
{
	const BVItem* a = (const BVItem*)va;
	const BVItem* b = (const BVItem*)vb;
	if (a->bmin[0] < b->bmin[0])
		return -1;
	if (a->bmin[0] > b->bmin[0])
		return 1;
	return 0;
}

static int compareItemY(const void* va, const void* vb)
{
	const BVItem* a = (const BVItem*)va;
	const BVItem* b = (const BVItem*)vb;
	if (a->bmin[1] < b->bmin[1])
		return -1;
	if (a->bmin[1] > b->bmin[1])
		return 1;
	return 0;
}

static int compareItemZ(const void* va, const void* vb)
{
	const BVItem* a = (const BVItem*)va;
	const BVItem* b = (const BVItem*)vb;
	if (a->bmin[2] < b->bmin[2])
		return -1;
	if (a->bmin[2] > b->bmin[2])
		return 1;
	return 0;
}

static void calcExtends(BVItem* items, const int /*nitems*/, const int imin, const int imax,
						unsigned short* bmin, unsigned short* bmax)
{
	bmin[0] = items[imin].bmin[0];
	bmin[1] = items[imin].bmin[1];
	bmin[2] = items[imin].bmin[2];
	
	bmax[0] = items[imin].bmax[0];
	bmax[1] = items[imin].bmax[1];
	bmax[2] = items[imin].bmax[2];
	
	for (int i = imin+1; i < imax; ++i)
	{
		const BVItem& it = items[i];
		if (it.bmin[0] < bmin[0]) bmin[0] = it.bmin[0];
		if (it.bmin[1] < bmin[1]) bmin[1] = it.bmin[1];
		if (it.bmin[2] < bmin[2]) bmin[2] = it.bmin[2];
		
		if (it.bmax[0] > bmax[0]) bmax[0] = it.bmax[0];
		if (it.bmax[1] > bmax[1]) bmax[1] = it.bmax[1];
		if (it.bmax[2] > bmax[2]) bmax[2] = it.bmax[2];
	}
}

inline int longestAxis(unsigned short x, unsigned short y, unsigned short z)
{
	int	axis = 0;
	unsigned short maxVal = x;
	if (y > maxVal)
	{
		axis = 1;
		maxVal = y;
	}
	if (z > maxVal)
	{
		axis = 2;
		maxVal = z;
	}
	return axis;
}

static void subdivide(BVItem* items, int nitems, int imin, int imax, int& curNode, dtBVNode* nodes)
{
	int inum = imax - imin;
	int icur = curNode;
	
	dtBVNode& node = nodes[curNode++];
	
	if (inum == 1)
	{
		// Leaf
		node.bmin[0] = items[imin].bmin[0];
		node.bmin[1] = items[imin].bmin[1];
		node.bmin[2] = items[imin].bmin[2];
		
		node.bmax[0] = items[imin].bmax[0];
		node.bmax[1] = items[imin].bmax[1];
		node.bmax[2] = items[imin].bmax[2];
		
		node.i = items[imin].i;
	}
	else
	{
		// Split
		calcExtends(items, nitems, imin, imax, node.bmin, node.bmax);
		
		int	axis = longestAxis(node.bmax[0] - node.bmin[0],
							   node.bmax[1] - node.bmin[1],
							   node.bmax[2] - node.bmin[2]);
		
		if (axis == 0)
		{
			// Sort along x-axis
			qsort(items+imin, inum, sizeof(BVItem), compareItemX);
		}
		else if (axis == 1)
		{
			// Sort along y-axis
			qsort(items+imin, inum, sizeof(BVItem), compareItemY);
		}
		else
		{
			// Sort along z-axis
			qsort(items+imin, inum, sizeof(BVItem), compareItemZ);
		}
		
		int isplit = imin+inum/2;
		
		// Left
		subdivide(items, nitems, imin, isplit, curNode, nodes);
		// Right
		subdivide(items, nitems, isplit, imax, curNode, nodes);
		
		int iescape = curNode - icur;
		// Negative index means escape.
		node.i = -iescape;
	}
}

static int createBVTree(const unsigned short* verts, const int /*nverts*/,
						const unsigned short* polys, const int npolys, const int nvp,
						const dtPolyDetail* DMeshes, const float* DVerts, const unsigned char* DTris, const float* tbmin,
						const float cs, const float ch,
						const int /*nnodes*/, dtBVNode* nodes)
{
	// Build tree
	BVItem* items = (BVItem*)dtAlloc(sizeof(BVItem)*npolys, DT_ALLOC_TEMP);
	for (int i = 0; i < npolys; i++)
	{
		BVItem& it = items[i];
		it.i = i;
		// Calc polygon bounds.
		const unsigned short* p = &polys[i*nvp*2];
		it.bmin[0] = it.bmax[0] = verts[p[0]*3+0];
		it.bmin[1] = it.bmax[1] = verts[p[0]*3+1];
		it.bmin[2] = it.bmax[2] = verts[p[0]*3+2];
		
		int vertCount = 0;
		for (int j = 1; j < nvp; ++j)
		{
			if (p[j] == MESH_NULL_IDX)
			{
				vertCount = j;
				break;
			}

			unsigned short x = verts[p[j]*3+0];
			unsigned short y = verts[p[j]*3+1];
			unsigned short z = verts[p[j]*3+2];
			
			if (x < it.bmin[0]) it.bmin[0] = x;
			if (y < it.bmin[1]) it.bmin[1] = y;
			if (z < it.bmin[2]) it.bmin[2] = z;
			
			if (x > it.bmax[0]) it.bmax[0] = x;
			if (y > it.bmax[1]) it.bmax[1] = y;
			if (z > it.bmax[2]) it.bmax[2] = z;
		}

		// include y from detail mesh
		const dtPolyDetail* pd = &DMeshes[i];
		for (int k = 0; k < pd->triCount; ++k)
		{
			const unsigned char* t = &DTris[(pd->triBase + k) * 4];
			for (int m = 0; m < 3; ++m)
			{
				if (t[m] >= vertCount)
				{
					const float* detailCoords = &DVerts[(pd->vertBase + (t[m] - vertCount)) * 3];
					const float qY = (detailCoords[1] - tbmin[1]) / ch;
					const unsigned short qYmin = (unsigned short)floorf(qY);
					const unsigned short qYmax = (unsigned short)ceilf(qY);

					if (qYmin < it.bmin[1]) it.bmin[1] = qYmin;
					if (qYmax > it.bmax[1]) it.bmax[1] = qYmax;
				}
			}
		}

		// Remap y
		it.bmin[1] = (unsigned short)floorf((float)it.bmin[1]*ch/cs);
		it.bmax[1] = (unsigned short)ceilf((float)it.bmax[1]*ch/cs);
	}
	
	int curNode = 0;
	subdivide(items, npolys, 0, npolys, curNode, nodes);
	
	dtFree(items);
	
	return curNode;
}

static unsigned char classifyOffMeshPoint(const float* pt, const float* bmin, const float* bmax)
{
	static const unsigned char XP = 1<<0;
	static const unsigned char ZP = 1<<1;
	static const unsigned char XM = 1<<2;
	static const unsigned char ZM = 1<<3;	

	unsigned char outcode = 0; 
	outcode |= (pt[0] >= bmax[0]) ? XP : 0;
	outcode |= (pt[2] >= bmax[2]) ? ZP : 0;
	outcode |= (pt[0] < bmin[0])  ? XM : 0;
	outcode |= (pt[2] < bmin[2])  ? ZM : 0;

	switch (outcode)
	{
	case XP: return 0;
	case XP|ZP: return 1;
	case ZP: return 2;
	case XM|ZP: return 3;
	case XM: return 4;
	case XM|ZM: return 5;
	case ZM: return 6;
	case XP|ZM: return 7;
	};

	return 0xff;	
}

// TODO: Better error handling.

/// @par
/// 
/// The output data array is allocated using the detour allocator (dtAlloc()).  The method
/// used to free the memory will be determined by how the tile is added to the navigation
/// mesh.
///
/// @see dtNavMesh, dtNavMesh::addTile()
bool dtCreateNavMeshData(dtNavMeshCreateParams* params, unsigned char** outData, int* outDataSize)
{
	if (params->nvp > DT_VERTS_PER_POLYGON)
		return false;
	if (params->vertCount >= 0xffff)
		return false;
	if (!params->vertCount || !params->verts)
		return false;
	if (!params->polyCount || !params->polys)
		return false;

	const int nvp = params->nvp;
	
	// Classify off-mesh connection points. We store only the connections
	// whose start point is inside the tile.
	unsigned char* offMeshConClass = 0;
	int storedOffMeshConCount = 0;
	int offMeshConLinkCount = 0;
	int storedOffMeshSegCount = 0;

	if (params->offMeshConCount > 0)
	{
		offMeshConClass = (unsigned char*)dtAlloc(sizeof(unsigned char)*params->offMeshConCount*2, DT_ALLOC_TEMP);
		if (!offMeshConClass)
			return false;

		memset(offMeshConClass, 0, sizeof(unsigned char)*params->offMeshConCount*2);

		// Find tight heigh bounds, used for culling out off-mesh start locations.
		float hmin = FLT_MAX;
		float hmax = -FLT_MAX;
		
		for (int i = 0; i < params->vertCount; ++i)
		{
			const unsigned short* iv = &params->verts[i*3];
			const float h = params->bmin[1] + iv[1] * params->ch;
			hmin = dtMin(hmin,h);
			hmax = dtMax(hmax,h);
		}

		if (params->detailVerts && params->detailVertsCount)
		{
			for (int i = 0; i < params->detailVertsCount; ++i)
			{
				const float h = params->detailVerts[i*3+1];
				hmin = dtMin(hmin,h);
				hmax = dtMax(hmax,h);
			}
		}

		hmin -= params->walkableClimb;
		hmax += params->walkableClimb;

		float bmin[3], bmax[3];
		dtVcopy(bmin, params->bmin);
		dtVcopy(bmax, params->bmax);
		bmin[1] = hmin;
		bmax[1] = hmax;
		
		float bverts[3*4];
		bverts[ 0] = bmin[0]; bverts[ 2] = bmin[2];
		bverts[ 3] = bmax[0]; bverts[ 5] = bmin[2];
		bverts[ 6] = bmax[0]; bverts[ 8] = bmax[2];
		bverts[ 9] = bmin[0]; bverts[11] = bmax[2];

		for (int i = 0; i < params->offMeshConCount; ++i)
		{
			const dtOffMeshLinkCreateParams& offMeshCon = params->offMeshCons[i];
			if (offMeshCon.type & DT_OFFMESH_CON_POINT)
			{
				offMeshConClass[i*2+0] = classifyOffMeshPoint(offMeshCon.vertsA0, bmin, bmax);
				offMeshConClass[i*2+1] = classifyOffMeshPoint(offMeshCon.vertsB0, bmin, bmax);

				// Zero out off-mesh start positions which are not even potentially touching the mesh.
				if (offMeshConClass[i*2+0] == 0xff)
				{
					if ((offMeshCon.vertsA0[1] - offMeshCon.snapHeight) > bmax[1] ||
						(offMeshCon.vertsA0[1] + offMeshCon.snapHeight) < bmin[1])
					{
						offMeshConClass[i * 2 + 0] = 0;
					}
				}

				// Count how many links should be allocated for off-mesh connections.
				if (offMeshConClass[i*2+0] == 0xff)
					offMeshConLinkCount++;
				if (offMeshConClass[i*2+1] == 0xff)
					offMeshConLinkCount++;

				if (offMeshConClass[i*2+0] == 0xff)
					storedOffMeshConCount++;
			}
			else if (offMeshCon.type & DT_OFFMESH_CON_SEGMENT)
			{
				int smin, smax;
				float tmin, tmax;
				if ((offMeshCon.vertsA0[1] >= bmin[1] && offMeshCon.vertsA0[1] <= bmax[1] && classifyOffMeshPoint(offMeshCon.vertsA0, bmin, bmax) == 0xff) ||
					(offMeshCon.vertsA1[1] >= bmin[1] && offMeshCon.vertsA1[1] <= bmax[1] && classifyOffMeshPoint(offMeshCon.vertsA1, bmin, bmax) == 0xff) ||
					(offMeshCon.vertsB0[1] >= bmin[1] && offMeshCon.vertsB0[1] <= bmax[1] && classifyOffMeshPoint(offMeshCon.vertsB0, bmin, bmax) == 0xff) ||
					(offMeshCon.vertsB1[1] >= bmin[1] && offMeshCon.vertsB1[1] <= bmax[1] && classifyOffMeshPoint(offMeshCon.vertsB1, bmin, bmax) == 0xff) ||
					dtIntersectSegmentPoly2D(offMeshCon.vertsA0, offMeshCon.vertsA1, bverts, 4, tmin, tmax, smin, smax) ||
					dtIntersectSegmentPoly2D(offMeshCon.vertsB0, offMeshCon.vertsB1, bverts, 4, tmin, tmax, smin, smax))
				{
					offMeshConClass[i*2] = 0xff;
					storedOffMeshSegCount++;
				}
			}
		}
	}
	
	// Off-mesh connections are stored as polygons, adjust values.
	const int firstSegVert = params->vertCount + storedOffMeshConCount*2;
	const int firstSegPoly = params->polyCount + storedOffMeshConCount;
	const int totPolyCount = firstSegPoly + storedOffMeshSegCount*DT_MAX_OFFMESH_SEGMENT_PARTS;
	const int totVertCount = firstSegVert + storedOffMeshSegCount*DT_MAX_OFFMESH_SEGMENT_PARTS*4;
	
	// Find portal edges which are at tile borders.
	int edgeCount = 0;
	int portalCount = 0;
	for (int i = 0; i < params->polyCount; ++i)
	{
		const unsigned short* p = &params->polys[i*2*nvp];
		for (int j = 0; j < nvp; ++j)
		{
			if (p[j] == MESH_NULL_IDX) break;
			edgeCount++;
			
			if (p[nvp+j] & 0x8000)
			{
				unsigned short dir = p[nvp+j] & 0xf;
				if (dir != 0xf)
					portalCount++;
			}
		}
	}

	// offmesh links will be added in dynamic array
	const int maxLinkCount = edgeCount + portalCount*2;
	
	// Find unique detail vertices.
	int uniqueDetailVertCount = 0;
	int detailTriCount = 0;
	if (params->detailMeshes)
	{
		// Has detail mesh, count unique detail vertex count and use input detail tri count.
		detailTriCount = params->detailTriCount;
		for (int i = 0; i < params->polyCount; ++i)
		{
			const unsigned short* p = &params->polys[i*nvp*2];
			int ndv = params->detailMeshes[i*4+1];
			int nv = 0;
			for (int j = 0; j < nvp; ++j)
			{
				if (p[j] == MESH_NULL_IDX) break;
				nv++;
			}
			ndv -= nv;
			uniqueDetailVertCount += ndv;
		}
	}
	else
	{
		// No input detail mesh, build detail mesh from nav polys.
		uniqueDetailVertCount = 0; // No extra detail verts.
		detailTriCount = 0;
		for (int i = 0; i < params->polyCount; ++i)
		{
			const unsigned short* p = &params->polys[i*nvp*2];
			int nv = 0;
			for (int j = 0; j < nvp; ++j)
			{
				if (p[j] == MESH_NULL_IDX) break;
				nv++;
			}
			detailTriCount += nv-2;
		}
	}
 
	// Calculate data size
	const int headerSize = dtAlign4(sizeof(dtMeshHeader));
	const int vertsSize = dtAlign4(sizeof(float)*3*totVertCount);
	const int polysSize = dtAlign4(sizeof(dtPoly)*totPolyCount);
	const int linksSize = dtAlign4(sizeof(dtLink)*maxLinkCount);
	const int detailMeshesSize = dtAlign4(sizeof(dtPolyDetail)*params->polyCount);
	const int detailVertsSize = dtAlign4(sizeof(float)*3*uniqueDetailVertCount);
	const int detailTrisSize = dtAlign4(sizeof(unsigned char)*4*detailTriCount);
	const int bvTreeSize = params->buildBvTree ? dtAlign4(sizeof(dtBVNode)*params->polyCount*2) : 0;
	const int offMeshConsSize = dtAlign4(sizeof(dtOffMeshConnection)*storedOffMeshConCount);
	const int offMeshSegSize = dtAlign4(sizeof(dtOffMeshSegmentConnection)*storedOffMeshSegCount);
	const int clustersSize = dtAlign4(sizeof(dtCluster)*params->clusterCount);
	const int polyClustersSize = dtAlign4(sizeof(unsigned short)*params->polyCount);
 
	
	const int dataSize = headerSize + vertsSize + polysSize + linksSize +
						 detailMeshesSize + detailVertsSize + detailTrisSize +
						 bvTreeSize + offMeshConsSize + offMeshSegSize +
						 clustersSize + polyClustersSize;
						 
	unsigned char* data = (unsigned char*)dtAlloc(sizeof(unsigned char)*dataSize, DT_ALLOC_PERM);
	if (!data)
	{
		dtFree(offMeshConClass);
		return false;
	}
	memset(data, 0, dataSize);
	
	unsigned char* d = data;
	dtMeshHeader* header = (dtMeshHeader*)d; d += headerSize;
	float* navVerts = (float*)d; d += vertsSize;
	dtPoly* navPolys = (dtPoly*)d; d += polysSize;
	d += linksSize;
	dtPolyDetail* navDMeshes = (dtPolyDetail*)d; d += detailMeshesSize;
	float* navDVerts = (float*)d; d += detailVertsSize;
	unsigned char* navDTris = (unsigned char*)d; d += detailTrisSize;
	dtBVNode* navBvtree = (dtBVNode*)d; d += bvTreeSize;
	dtOffMeshConnection* offMeshCons = (dtOffMeshConnection*)d; d += offMeshConsSize;
	dtOffMeshSegmentConnection* offMeshSegs = (dtOffMeshSegmentConnection*)d; d += offMeshSegSize;
	dtCluster* clusters = (dtCluster*)d; d += clustersSize;
	unsigned short* polyClusters = (unsigned short*)d; d += polyClustersSize;
	
	// Store header
	header->magic = DT_NAVMESH_MAGIC;
	header->version = DT_NAVMESH_VERSION;
	header->x = params->tileX;
	header->y = params->tileY;
	header->layer = params->tileLayer;
	header->userId = params->userId;
	header->polyCount = totPolyCount;
	header->vertCount = totVertCount;
	header->maxLinkCount = maxLinkCount;
	dtVcopy(header->bmin, params->bmin);
	dtVcopy(header->bmax, params->bmax);
	header->detailMeshCount = params->polyCount;
	header->detailVertCount = uniqueDetailVertCount;
	header->detailTriCount = detailTriCount;
	header->bvQuantFactor = 1.0f / params->cs;
	header->offMeshBase = params->polyCount;
	header->offMeshSegPolyBase = firstSegPoly;
	header->offMeshSegVertBase = firstSegVert;
	header->walkableHeight = params->walkableHeight;
	header->walkableRadius = params->walkableRadius;
	header->walkableClimb = params->walkableClimb;
	header->offMeshConCount = storedOffMeshConCount;
	header->offMeshSegConCount = storedOffMeshSegCount;
	header->bvNodeCount = params->buildBvTree ? params->polyCount*2 : 0;
	header->clusterCount = params->clusterCount; 

	const int offMeshVertsBase = params->vertCount;
	const int offMeshPolyBase = params->polyCount;

	// Store vertices
	// Mesh vertices
	for (int i = 0; i < params->vertCount; ++i)
	{
		const unsigned short* iv = &params->verts[i*3];
		float* v = &navVerts[i*3];
		v[0] = params->bmin[0] + iv[0] * params->cs;
		v[1] = params->bmin[1] + iv[1] * params->ch;
		v[2] = params->bmin[2] + iv[2] * params->cs;
	}
	// Off-mesh point link vertices.
	int n = 0;
	for (int i = 0; i < params->offMeshConCount; ++i)
	{
		const dtOffMeshLinkCreateParams& offMeshCon = params->offMeshCons[i];

		// Only store connections which start from this tile.
		if ((offMeshConClass[i*2+0] == 0xff) && (offMeshCon.type & DT_OFFMESH_CON_POINT))
		{
			float* v = &navVerts[(offMeshVertsBase + n*2)*3];
			dtVcopy(&v[0], &offMeshCon.vertsA0[0]);
			dtVcopy(&v[3], &offMeshCon.vertsB0[0]);
			n++;
		}
	}

	// Store polygons
	// Mesh polys
	const unsigned short* src = params->polys;
	for (int i = 0; i < params->polyCount; ++i)
	{
		dtPoly* p = &navPolys[i];
		p->vertCount = 0;
		p->flags = params->polyFlags[i];
		p->setArea(params->polyAreas[i]);
		p->setType(DT_POLYTYPE_GROUND);
		for (int j = 0; j < nvp; ++j)
		{
			if (src[j] == MESH_NULL_IDX) break;
			p->verts[j] = src[j];
			if (src[nvp+j] & 0x8000)
			{
				// Border or portal edge.
				unsigned short dir = src[nvp+j] & 0xf;
				if (dir == 0xf) // Border
					p->neis[j] = 0;
				else if (dir == 0) // Portal x-
					p->neis[j] = DT_EXT_LINK | 4;
				else if (dir == 1) // Portal z+
					p->neis[j] = DT_EXT_LINK | 2;
				else if (dir == 2) // Portal x+
					p->neis[j] = DT_EXT_LINK | 0;
				else if (dir == 3) // Portal z-
					p->neis[j] = DT_EXT_LINK | 6;
			}
			else
			{
				// Normal connection
				p->neis[j] = src[nvp+j]+1;
			}
			
			p->vertCount++;
		}
		src += nvp*2;
	}
	// Off-mesh point connection polygons.
	n = 0;
	int nseg = 0;
	for (int i = 0; i < params->offMeshConCount; ++i)
	{
		const dtOffMeshLinkCreateParams& offMeshCon = params->offMeshCons[i];

		// Only store connections which start from this tile.
		if (offMeshConClass[i*2+0] == 0xff)
		{
			if (offMeshCon.type & DT_OFFMESH_CON_POINT)
			{
				dtPoly* p = &navPolys[offMeshPolyBase+n];
				p->vertCount = 2;
				p->verts[0] = (unsigned short)(offMeshVertsBase + n*2+0);
				p->verts[1] = (unsigned short)(offMeshVertsBase + n*2+1);
				p->flags = offMeshCon.polyFlag;
				p->setArea(offMeshCon.area);
				p->setType(DT_POLYTYPE_OFFMESH_POINT);
				n++;
			}
			else
			{
				for (int j = 0; j < DT_MAX_OFFMESH_SEGMENT_PARTS; j++)
				{
					dtPoly* p = &navPolys[firstSegPoly+nseg];
					p->vertCount = 0;
					p->flags = offMeshCon.polyFlag;
					p->setArea(offMeshCon.area);
					p->setType(DT_POLYTYPE_OFFMESH_SEGMENT);
					nseg++;
				}
			}
		}
	}

	// Store detail meshes and vertices.
	// The nav polygon vertices are stored as the first vertices on each mesh.
	// We compress the mesh data by skipping them and using the navmesh coordinates.
	if (params->detailMeshes)
	{
		unsigned short vbase = 0;
		for (int i = 0; i < params->polyCount; ++i)
		{
			dtPolyDetail& dtl = navDMeshes[i];
			const int vb = (int)params->detailMeshes[i*4+0];
			const int ndv = (int)params->detailMeshes[i*4+1];
			const int nv = navPolys[i].vertCount;
			dtl.vertBase = (unsigned int)vbase;
			dtl.vertCount = (unsigned char)(ndv-nv);
			dtl.triBase = (unsigned int)params->detailMeshes[i*4+2];
			dtl.triCount = (unsigned char)params->detailMeshes[i*4+3];
			// Copy vertices except the first 'nv' verts which are equal to nav poly verts.
			if (ndv-nv)
			{
				memcpy(&navDVerts[vbase*3], &params->detailVerts[(vb+nv)*3], sizeof(float)*3*(ndv-nv));
				vbase += (unsigned short)(ndv-nv);
			}
		}
		// Store triangles.
		memcpy(navDTris, params->detailTris, sizeof(unsigned char)*4*params->detailTriCount);
	}
	else
	{
		// Create dummy detail mesh by triangulating polys.
		int tbase = 0;
		for (int i = 0; i < params->polyCount; ++i)
		{
			dtPolyDetail& dtl = navDMeshes[i];
			const int nv = navPolys[i].vertCount;
			dtl.vertBase = 0;
			dtl.vertCount = 0;
			dtl.triBase = (unsigned int)tbase;
			dtl.triCount = (unsigned char)(nv-2);
			// Triangulate polygon (local indices).
			for (int j = 2; j < nv; ++j)
			{
				unsigned char* t = &navDTris[tbase*4];
				t[0] = 0;
				t[1] = (unsigned char)(j-1);
				t[2] = (unsigned char)j;
				// Bit for each edge that belongs to poly boundary.
				t[3] = (1<<2);
				if (j == 2) t[3] |= (1<<0);
				if (j == nv-1) t[3] |= (1<<4);
				tbase++;
			}
		}
	}

	// Store and create BVtree.
	if (params->buildBvTree)
	{
		createBVTree(params->verts, params->vertCount, params->polys, params->polyCount, nvp,
					 navDMeshes, navDVerts, navDTris, params->bmin,
					 params->cs, params->ch, params->polyCount*2, navBvtree);
	}
	
	// Store Off-Mesh connections.
	n = 0;
	nseg = 0;
	for (int i = 0; i < params->offMeshConCount; ++i)
	{
		const dtOffMeshLinkCreateParams& offMeshCon = params->offMeshCons[i];

		// Only store connections which start from this tile.
		if (offMeshConClass[i*2+0] == 0xff)
		{
			if (offMeshCon.type & DT_OFFMESH_CON_POINT)
			{
				dtOffMeshConnection* con = &offMeshCons[n];
				con->poly = (unsigned short)(offMeshPolyBase + n);
				// Copy connection end-points.
				dtVcopy(&con->pos[0], &offMeshCon.vertsA0[0]);
				dtVcopy(&con->pos[3], &offMeshCon.vertsB0[0]);
				con->rad = offMeshCon.snapRadius;
				con->height = offMeshCon.snapHeight;
				con->setFlags(offMeshCon.type);
				con->side = offMeshConClass[i*2+1] == 0xff ? DT_CONNECTION_INTERNAL : offMeshConClass[i*2+1];
				if (offMeshCon.userID)
					con->userId = offMeshCon.userID;
				n++;
			}
			else
			{
				dtOffMeshSegmentConnection* con = &offMeshSegs[nseg];
				dtVcopy(con->startA, &offMeshCon.vertsA0[0]);
				dtVcopy(con->endA, &offMeshCon.vertsA1[0]);
				dtVcopy(con->startB, &offMeshCon.vertsB0[0]);
				dtVcopy(con->endB, &offMeshCon.vertsB1[0]);

				con->rad = offMeshCon.snapRadius;
				con->height = offMeshCon.snapHeight;
				con->setFlags(offMeshCon.type);
				if (offMeshCon.userID)
					con->userId = offMeshCon.userID;

				nseg++;
			}
		}
	}

	dtFree(offMeshConClass);
	
	// Store clusters
	if (params->polyClusters)
	{
		memcpy(polyClusters, params->polyClusters, sizeof(unsigned short)*params->polyCount);
	}

	for (int i = 0; i < params->clusterCount; i++)
	{
		dtCluster& cluster = clusters[i];
		cluster.firstLink = DT_NULL_LINK;
		cluster.numLinks = 0;
		dtVset(cluster.center, 0.f, 0.f, 0.f);

		// calculate center point: take from first poly
		for (int j = 0; j < params->polyCount; j++)
		{
			if (polyClusters[j] != i)
			{
				continue;
			}

			const dtPoly* poly = &navPolys[j];
			float c[3] = { 0.0f, 0.0f, 0.0f };

			for (int iv = 0; iv < poly->vertCount; iv++)
			{
				dtVadd(c, c, &navVerts[poly->verts[iv] * 3]);
			}
			
			dtVmad(cluster.center, cluster.center, c, 1.0f / poly->vertCount);
			break;
		}
	}
 
	*outData = data;
	*outDataSize = dataSize;
	
	return true;
}

bool dtNavMeshHeaderSwapEndian(unsigned char* data, const int /*dataSize*/)
{
	dtMeshHeader* header = (dtMeshHeader*)data;
	
	int swappedMagic = DT_NAVMESH_MAGIC;
	int swappedVersion = DT_NAVMESH_VERSION;
	dtSwapEndian(&swappedMagic);
	dtSwapEndian(&swappedVersion);
	
	if ((header->magic != DT_NAVMESH_MAGIC || header->version != DT_NAVMESH_VERSION) &&
		(header->magic != swappedMagic || header->version != swappedVersion))
	{
		return false;
	}
		
	dtSwapEndian(&header->magic);
	dtSwapEndian(&header->version);
	dtSwapEndian(&header->x);
	dtSwapEndian(&header->y);
	dtSwapEndian(&header->layer);
	dtSwapEndian(&header->userId);
	dtSwapEndian(&header->polyCount);
	dtSwapEndian(&header->vertCount);
	dtSwapEndian(&header->maxLinkCount);
	dtSwapEndian(&header->detailMeshCount);
	dtSwapEndian(&header->detailVertCount);
	dtSwapEndian(&header->detailTriCount);
	dtSwapEndian(&header->bvNodeCount);
	dtSwapEndian(&header->offMeshConCount);
	dtSwapEndian(&header->offMeshSegConCount);
	dtSwapEndian(&header->offMeshBase);
	dtSwapEndian(&header->offMeshSegPolyBase);
	dtSwapEndian(&header->offMeshSegVertBase);
	dtSwapEndian(&header->walkableHeight);
	dtSwapEndian(&header->walkableRadius);
	dtSwapEndian(&header->walkableClimb);
	dtSwapEndian(&header->bmin[0]);
	dtSwapEndian(&header->bmin[1]);
	dtSwapEndian(&header->bmin[2]);
	dtSwapEndian(&header->bmax[0]);
	dtSwapEndian(&header->bmax[1]);
	dtSwapEndian(&header->bmax[2]);
	dtSwapEndian(&header->bvQuantFactor);
	dtSwapEndian(&header->clusterCount);

	// Freelist index and pointers are updated when tile is added, no need to swap.

	return true;
}

/// @par
///
/// @warning This function assumes that the header is in the correct endianess already. 
/// Call #dtNavMeshHeaderSwapEndian() first on the data if the data is expected to be in wrong endianess 
/// to start with. Call #dtNavMeshHeaderSwapEndian() after the data has been swapped if converting from 
/// native to foreign endianess.
bool dtNavMeshDataSwapEndian(unsigned char* data, const int /*dataSize*/)
{
	// Make sure the data is in right format.
	dtMeshHeader* header = (dtMeshHeader*)data;
	if (header->magic != DT_NAVMESH_MAGIC)
		return false;
	if (header->version != DT_NAVMESH_VERSION)
		return false;
	
	// Patch header pointers.
	const int headerSize = dtAlign4(sizeof(dtMeshHeader));
	const int vertsSize = dtAlign4(sizeof(float)*3*header->vertCount);
	const int polysSize = dtAlign4(sizeof(dtPoly)*header->polyCount);
	const int linksSize = dtAlign4(sizeof(dtLink)*(header->maxLinkCount));
	const int detailMeshesSize = dtAlign4(sizeof(dtPolyDetail)*header->detailMeshCount);
	const int detailVertsSize = dtAlign4(sizeof(float)*3*header->detailVertCount);
	const int detailTrisSize = dtAlign4(sizeof(unsigned char)*4*header->detailTriCount);
	const int bvtreeSize = dtAlign4(sizeof(dtBVNode)*header->bvNodeCount);
	const int offMeshLinksSize = dtAlign4(sizeof(dtOffMeshConnection)*header->offMeshConCount);
	const int offMeshSegSize = dtAlign4(sizeof(dtOffMeshSegmentConnection)*header->offMeshSegConCount);
	const int clustersSize = dtAlign4(sizeof(dtCluster)*header->clusterCount);
	const int polyClustersSize = dtAlign4(sizeof(unsigned short)*header->offMeshBase);

	unsigned char* d = data + headerSize;
	float* verts = (float*)d; d += vertsSize;
	dtPoly* polys = (dtPoly*)d; d += polysSize;
	/*dtLink* links = (dtLink*)d;*/ d += linksSize;
	dtPolyDetail* detailMeshes = (dtPolyDetail*)d; d += detailMeshesSize;
	float* detailVerts = (float*)d; d += detailVertsSize;
	/*unsigned char* detailTris = (unsigned char*)d;*/ d += detailTrisSize;
	dtBVNode* bvTree = (dtBVNode*)d; d += bvtreeSize;
	dtOffMeshConnection* offMeshCons = (dtOffMeshConnection*)d; d += offMeshLinksSize;
	dtOffMeshSegmentConnection* offMeshSegs = (dtOffMeshSegmentConnection*)d; d += offMeshSegSize;
	dtCluster* clusters = (dtCluster*)d; d += clustersSize;
	unsigned short* polyClusters = (unsigned short*)d; d += polyClustersSize;
 	
	// Vertices
	for (int i = 0; i < header->vertCount*3; ++i)
	{
		dtSwapEndian(&verts[i]);
	}

	// Polys
	for (int i = 0; i < header->polyCount; ++i)
	{
		dtPoly* p = &polys[i];
		// poly->firstLink is update when tile is added, no need to swap.
		for (int j = 0; j < DT_VERTS_PER_POLYGON; ++j)
		{
			dtSwapEndian(&p->verts[j]);
			dtSwapEndian(&p->neis[j]);
		}
		dtSwapEndian(&p->flags);
	}

	// Links are rebuild when tile is added, no need to swap.

	// Detail meshes
	for (int i = 0; i < header->detailMeshCount; ++i)
	{
		dtPolyDetail* pd = &detailMeshes[i];
		dtSwapEndian(&pd->vertBase);
		dtSwapEndian(&pd->triBase);
	}
	
	// Detail verts
	for (int i = 0; i < header->detailVertCount*3; ++i)
	{
		dtSwapEndian(&detailVerts[i]);
	}

	// BV-tree
	for (int i = 0; i < header->bvNodeCount; ++i)
	{
		dtBVNode* node = &bvTree[i];
		for (int j = 0; j < 3; ++j)
		{
			dtSwapEndian(&node->bmin[j]);
			dtSwapEndian(&node->bmax[j]);
		}
		dtSwapEndian(&node->i);
	}

	// Off-mesh Connections: point type
	for (int i = 0; i < header->offMeshConCount; ++i)
	{
		dtOffMeshConnection* con = &offMeshCons[i];
		for (int j = 0; j < 6; ++j)
			dtSwapEndian(&con->pos[j]);
		dtSwapEndian(&con->rad);
		dtSwapEndian(&con->poly);
		dtSwapEndian(&con->userId);
	}

	// Off-mesh Connections: segment type
	for (int i = 0; i < header->offMeshSegConCount; ++i)
	{
		dtOffMeshSegmentConnection* con = &offMeshSegs[i];
		for (int j = 0; j < 3; ++j)
		{
			dtSwapEndian(&con->startA[j]);
			dtSwapEndian(&con->startB[j]);
			dtSwapEndian(&con->endA[j]);
			dtSwapEndian(&con->endB[j]);
		}
		dtSwapEndian(&con->rad);
		dtSwapEndian(&con->firstPoly);
		dtSwapEndian(&con->userId);
	}

	for (int i = 0; i < header->offMeshBase; i++)
	{
		dtSwapEndian(&polyClusters[i]);
	}
 
	return true;
}
