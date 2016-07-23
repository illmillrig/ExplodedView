#pragma once

#include <maya/MPxDeformerNode.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MTypeId.h>
#include <maya/MPointArray.h>
#include <maya/MVectorArray.h>
#include <maya/MFloatVectorArray.h>

#include <vector>
#include <set>


class ExplodedView : public MPxDeformerNode {
public:
    ExplodedView();
    ~ExplodedView();
	virtual MPxNode::SchedulingType schedulingType() const override { return MPxNode::kParallel; }
	static void* creator();
	static MStatus initialize();
	virtual MStatus compute (const MPlug &plug, MDataBlock &data) override;
	virtual MStatus deformPoints (MDataBlock &data);

	MStatus computeShells(const MPlug &plug, MDataBlock &data, MStatus &stat);
	MStatus collectVertexNeighbors(MObject &inMesh, std::vector<std::vector<int>> &vertNeighborCache, MStatus &stat);
	MStatus collectShells(const std::vector<std::vector<int>> &vertNeighborCache, MArrayDataHandle &shellsHandle, MStatus &stat);
	void diffNeighbors(std::set<int> &setA, const std::set<int> &setB, std::set<int> &destination) const;
	MStatus collectShellDeltas(MObject &inMesh, MArrayDataHandle &shellsHandle);
	void avgShellPoints(const MPointArray &positions);
	MStatus processPositions(const double &distance, const MVector &spread, const MVector &push, const MMatrix &spin,
							 const MVector &scale, MObject &inMesh, MObject &outMesh);

public:
	static MTypeId id;
	static MObject distance;
	static MObject spread;
	static MObject spreadX;
	static MObject spreadY;
	static MObject spreadZ;
	static MObject slide;
	static MObject slideX;
	static MObject slideY;
	static MObject slideZ;
	static MObject spin;
	static MObject spinX;
	static MObject spinY;
	static MObject spinZ;
	static MObject scale;
	static MObject scaleX;
	static MObject scaleY;
	static MObject scaleZ;
	static MObject shells;

	MIntArray shellIDs;
	std::vector<MVector> shellDeltas;
	unsigned int numShells;

	MPointArray positions;
};
