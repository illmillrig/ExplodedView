
#include "ExplodedView.h"

#include <maya/MGlobal.h>
#include <maya/MFnMesh.h>
#include <maya/MFnUnitAttribute.h>
#include <maya/MItGeometry.h>
#include <maya/MItMeshVertex.h>
#include <maya/MEulerRotation.h>
#include <maya/MQuaternion.h>
#include <maya/MMatrix.h>
#include <maya/MFloatMatrix.h>

#include <algorithm>


MTypeId ExplodedView::id(0x001226D4); // this needs to be changed
MObject ExplodedView::distance;
MObject ExplodedView::spread;
MObject ExplodedView::spreadX;
MObject ExplodedView::spreadY;
MObject ExplodedView::spreadZ;
MObject ExplodedView::slide;
MObject ExplodedView::slideX;
MObject ExplodedView::slideY;
MObject ExplodedView::slideZ;
MObject ExplodedView::spin;
MObject ExplodedView::spinX;
MObject ExplodedView::spinY;
MObject ExplodedView::spinZ;
MObject ExplodedView::scale;
MObject ExplodedView::scaleX;
MObject ExplodedView::scaleY;
MObject ExplodedView::scaleZ;
MObject ExplodedView::shells;


ExplodedView::ExplodedView() { }
ExplodedView::~ExplodedView() { }

void* ExplodedView::creator() {
	return new ExplodedView();
}


MStatus ExplodedView::initialize() {
	// attributes are writable by default
	// attributes are storable by default
	// attributes are readable by default
	// attributes not keyable by default

	MStatus stat;
	MFnNumericAttribute fnNum;
	MFnUnitAttribute fnUnit;


	distance = fnNum.create("distance", "dist", MFnNumericData::kDouble, 0.0, &stat);
	CHECK_MSTATUS_AND_RETURN_IT(stat);
	fnNum.setKeyable(true);
	stat = ExplodedView::addAttribute(distance);
	CHECK_MSTATUS_AND_RETURN_IT(stat);


	spreadX = fnNum.create("spreadX", "sprX", MFnNumericData::kDouble, 0.0, &stat);
	CHECK_MSTATUS_AND_RETURN_IT(stat);
	fnNum.setKeyable(true);

	spreadY = fnNum.create("spreadY", "sprY", MFnNumericData::kDouble, 0.0, &stat);
	CHECK_MSTATUS_AND_RETURN_IT(stat);
	fnNum.setKeyable(true);

	spreadZ = fnNum.create("spreadZ", "sprZ", MFnNumericData::kDouble, 0.0, &stat);
	CHECK_MSTATUS_AND_RETURN_IT(stat);
	fnNum.setKeyable(true);

	spread = fnNum.create("spread", "sprd", spreadX, spreadY, spreadZ, &stat);
	CHECK_MSTATUS_AND_RETURN_IT(stat);
	fnNum.setKeyable(true);
	stat = ExplodedView::addAttribute(spread);
	CHECK_MSTATUS_AND_RETURN_IT(stat);


	slideX = fnNum.create("slideX", "sldX", MFnNumericData::kDouble, 0.0, &stat);
	CHECK_MSTATUS_AND_RETURN_IT(stat);
	fnNum.setKeyable(true);

	slideY = fnNum.create("slideY", "sldY", MFnNumericData::kDouble, 0.0, &stat);
	CHECK_MSTATUS_AND_RETURN_IT(stat);
	fnNum.setKeyable(true);

	slideZ = fnNum.create("slideZ", "sldZ", MFnNumericData::kDouble, 0.0, &stat);
	CHECK_MSTATUS_AND_RETURN_IT(stat);
	fnNum.setKeyable(true);

	slide = fnNum.create("slide", "slde", slideX, slideY, slideZ, &stat);
	CHECK_MSTATUS_AND_RETURN_IT(stat);
	fnNum.setKeyable(true);
	stat = ExplodedView::addAttribute(slide);
	CHECK_MSTATUS_AND_RETURN_IT(stat);


	spinX = fnUnit.create("spinX", "spnX", MFnUnitAttribute::kAngle, 0.0, &stat);
	CHECK_MSTATUS_AND_RETURN_IT(stat);
	fnUnit.setKeyable(true);

	spinY = fnUnit.create("spinY", "spnY", MFnUnitAttribute::kAngle, 0.0, &stat);
	CHECK_MSTATUS_AND_RETURN_IT(stat);
	fnUnit.setKeyable(true);

	spinZ = fnUnit.create("spinZ", "spnZ", MFnUnitAttribute::kAngle, 0.0, &stat);
	CHECK_MSTATUS_AND_RETURN_IT(stat);
	fnUnit.setKeyable(true);

	spin = fnNum.create("spin", "spin", spinX, spinY, spinZ, &stat);
	CHECK_MSTATUS_AND_RETURN_IT(stat);
	fnNum.setKeyable(true);
	stat = ExplodedView::addAttribute(spin);
	CHECK_MSTATUS_AND_RETURN_IT(stat);


	scaleX = fnNum.create("scaleX", "sclX", MFnNumericData::kDouble, 1.0, &stat);
	CHECK_MSTATUS_AND_RETURN_IT(stat);
	fnNum.setKeyable(true);

	scaleY = fnNum.create("scaleY", "sclY", MFnNumericData::kDouble, 1.0, &stat);
	CHECK_MSTATUS_AND_RETURN_IT(stat);
	fnNum.setKeyable(true);

	scaleZ = fnNum.create("scaleZ", "sclZ", MFnNumericData::kDouble, 1.0, &stat);
	CHECK_MSTATUS_AND_RETURN_IT(stat);
	fnNum.setKeyable(true);

	scale = fnNum.create("scale", "scl", scaleX, scaleY, scaleZ, &stat);
	CHECK_MSTATUS_AND_RETURN_IT(stat);
	fnNum.setKeyable(true);
	stat = ExplodedView::addAttribute(scale);
	CHECK_MSTATUS_AND_RETURN_IT(stat);


	shells = fnNum.create("shells", "shls", MFnNumericData::kInt, 0, &stat);
	CHECK_MSTATUS_AND_RETURN_IT(stat);
	fnNum.setStorable(false);
	fnNum.setWritable(false);
	fnNum.setArray(true);
	fnNum.setUsesArrayDataBuilder(true);
	stat = ExplodedView::addAttribute(shells);
	CHECK_MSTATUS_AND_RETURN_IT(stat);


	// TODO: Change this->numShells to an output int attribute on the node

	// TODO: change this to an output array attribute for both shellIDs and shellDeltas


	ExplodedView::attributeAffects(inputGeom, shells);
	ExplodedView::attributeAffects(inputGeom, outputGeom);
	ExplodedView::attributeAffects(distance, outputGeom);
	ExplodedView::attributeAffects(spread, outputGeom);
	ExplodedView::attributeAffects(slide, outputGeom);
	ExplodedView::attributeAffects(spin, outputGeom);
	ExplodedView::attributeAffects(scale, outputGeom);

	return stat;
}


MStatus ExplodedView::compute(const MPlug &plug, MDataBlock &data) {

	MStatus stat;

	// Compute
	if (plug == shells) {

		this->computeShells(plug, data, stat);
		CHECK_MSTATUS_AND_RETURN_IT(stat);

		data.setClean(plug);
		return MS::kSuccess;
	}


	// Deform
	else if (plug == outputGeom){

		if (!this->shellIDs.length()){
			MPlug shellsPlug (thisMObject(), ExplodedView::shells);

			stat = this->computeShells(shellsPlug, data, stat);
			CHECK_MSTATUS_AND_RETURN_IT(stat);
		}

		stat = this->deformPoints(data);
		CHECK_MSTATUS_AND_RETURN_IT(stat);

		data.setClean(plug);
		return MS::kSuccess;
	}

	return MS::kUnknownParameter;
}


MStatus ExplodedView::computeShells(const MPlug &plug, MDataBlock &data, MStatus &stat) {

	// TODO: make this work for multiple objects being deformed by one node.

	// inputs
	MArrayDataHandle inputHandle = data.inputArrayValue(input, &stat);
	CHECK_MSTATUS_AND_RETURN_IT(stat);

	MObject inMesh = inputHandle.inputValue().asMesh();

	// confirm inMesh is a polyMesh
	if (!inMesh.hasFn(MFn::kMesh))
		return MS::kUnknownParameter;

	// outputs
	MArrayDataHandle shellsHandle = data.outputArrayValue(shells, &stat);
	CHECK_MSTATUS_AND_RETURN_IT(stat);

	// create vertex neighbor cache
	std::vector<std::vector<int>> vertNeighborCache;
	stat = this->collectVertexNeighbors(inMesh, vertNeighborCache, stat);
	CHECK_MSTATUS_AND_RETURN_IT(stat);

	// walk mesh to find shells by vertex neighbors
	this->collectShells(vertNeighborCache, shellsHandle, stat);
	CHECK_MSTATUS_AND_RETURN_IT(stat);

	// create offset deltas from shell points
	stat = this->collectShellDeltas(inMesh, shellsHandle);
	CHECK_MSTATUS_AND_RETURN_IT(stat);

	return MS::kSuccess;
}


MStatus ExplodedView::collectVertexNeighbors(MObject &inMesh, std::vector<std::vector<int>> &vertNeighborCache, MStatus &stat) {

	MItMeshVertex iter (inMesh, &stat);
	CHECK_MSTATUS_AND_RETURN_IT(stat);

	vertNeighborCache.resize( (size_t) iter.count() );

	MIntArray neighbors;
	for ( ; !iter.isDone(); iter.next()) {
		iter.getConnectedVertices(neighbors);
		vertNeighborCache[iter.index()] = std::vector<int> (&neighbors[0], &neighbors[neighbors.length()]);
	}

	return MS::kSuccess;
}


MStatus ExplodedView::collectShells(const std::vector<std::vector<int>> &vertNeighborCache, MArrayDataHandle &shellsHandle, MStatus &stat) {

	unsigned int vertCount = (unsigned int) vertNeighborCache.size();
	this->shellIDs.setLength(vertCount);

	std::set<int> toBeVisited;
	std::set<int> neighbors;
	std::set<int> currentVertices;
	std::set<int> visitedVertices;
	std::set<int> difference;

	for (int i = 0; i < vertCount; ++i)
		toBeVisited.insert(i);

	this->numShells = 1;
	int currentShellID = 0;

	while (!toBeVisited.empty()){

		neighbors.insert( *(toBeVisited.begin()) );

		while (!neighbors.empty()){

			currentVertices = neighbors;
			visitedVertices.insert(currentVertices.begin(), currentVertices.end());

			for (const auto vertID : currentVertices){
				neighbors.insert(vertNeighborCache[vertID].begin(), vertNeighborCache[vertID].end());
				this->shellIDs[vertID] = currentShellID;
			}

			this->diffNeighbors(neighbors, visitedVertices, difference);
		}

		this->diffNeighbors(toBeVisited, visitedVertices, difference);

		currentShellID++;
		this->numShells++;
	}

	return MS::kSuccess;
}


void ExplodedView::diffNeighbors(std::set<int> &setA, const std::set<int> &setB, std::set<int> &destination) const {

	destination.clear();
	std::set_difference(setA.begin(), setA.end(),
						setB.begin(), setB.end(),
						std::inserter(destination, destination.end()) );
	setA.clear();
	setA.insert(destination.begin(), destination.end());
}


MStatus ExplodedView::collectShellDeltas(MObject &inMesh, MArrayDataHandle &shellsHandle) {
	MStatus stat;

	MFnMesh fnMesh (inMesh, &stat);
	CHECK_MSTATUS_AND_RETURN_IT(stat);

	MPointArray positions;
	fnMesh.getPoints(positions, MSpace::kObject);

	this->avgShellPoints(positions);

	return MS::kSuccess;
}


void ExplodedView::avgShellPoints(const MPointArray &positions) {

	this->shellDeltas.resize( (size_t)this->numShells );
	MIntArray shellNumPoints (this->numShells, 0);

	// add together point position for each shell
	int numPoints = positions.length();
	for (int i = 0; i < numPoints ; ++i) {
		this->shellDeltas[ this->shellIDs[i] ] += positions[i];
		++shellNumPoints[ this->shellIDs[i] ];
	}

	// average point position for each shell
	for (int j = 0; j < this->numShells; ++j)
		this->shellDeltas[j] /= shellNumPoints[j];
}


MStatus ExplodedView::deformPoints(MDataBlock &data) {
	MStatus stat;

	// TODO: make this work for deforming multiple objects

	double distance = data.outputValue(ExplodedView::distance).asDouble();
	MVector spread = data.outputValue(ExplodedView::spread).asVector();
	MVector push = data.outputValue(ExplodedView::slide).asVector();
	MEulerRotation spinRot = data.outputValue(ExplodedView::spin).asVector();
	MMatrix spin = spinRot.asMatrix();
	MVector scale = data.outputValue(ExplodedView::scale).asVector();



	MArrayDataHandle inHandle = data.inputArrayValue(input, &stat);
	CHECK_MSTATUS_AND_RETURN_IT(stat);
	MDataHandle inMeshHandle = inHandle.outputValue();
	MObject inMesh = inMeshHandle.asMesh();

	if (!inMesh.hasFn(MFn::kMesh))
		return MS::kUnknownParameter;



	MArrayDataHandle outHandle = data.outputArrayValue(outputGeom, &stat);
	CHECK_MSTATUS_AND_RETURN_IT(stat);
	MDataHandle outMeshHandle  = outHandle.outputValue();
	MObject outMesh = outMeshHandle.asMesh();


	if (outMesh.isNull()){
		stat = outMeshHandle.setMObject(inMesh);
		CHECK_MSTATUS_AND_RETURN_IT(stat);
		outMesh = outMeshHandle.asMesh();
	}


	if (!outMesh.hasFn(MFn::kMesh))
		return MS::kUnknownParameter;



	stat = this->processPositions(distance, spread, push, spin, scale, inMesh, outMesh);
	CHECK_MSTATUS_AND_RETURN_IT(stat);

	return MS::kSuccess;
}


MStatus ExplodedView::processPositions(const double &distance, const MVector &spread, const MVector &push, const MMatrix &spin,
									   const MVector &scale, MObject &inMesh, MObject &outMesh) {
	MStatus stat;

	MFnMesh fnMesh (inMesh, &stat);
	CHECK_MSTATUS_AND_RETURN_IT(stat);

	stat = fnMesh.getPoints(this->positions, MSpace::kObject);
	CHECK_MSTATUS_AND_RETURN_IT(stat);

	MVector offset;
	int numPoints = fnMesh.numVertices();
	for (unsigned int index = 0; index < numPoints; ++index) {

		offset = this->shellDeltas[ this->shellIDs[index] ];

		this->positions[index] -= this->shellDeltas[ this->shellIDs[index] ];
		this->positions[index] *= spin;

		this->positions[index].x *= scale.x;
		this->positions[index].y *= scale.y;
		this->positions[index].z *= scale.z;

		offset.x *= spread.x;
		offset.y *= spread.y;
		offset.z *= spread.z;

		offset += push;
		offset += (this->shellDeltas[ this->shellIDs[index] ] * distance);

		this->positions[index] += this->shellDeltas[ this->shellIDs[index] ] + offset;
	}

	stat = fnMesh.setObject(outMesh);
	CHECK_MSTATUS_AND_RETURN_IT(stat);

	stat = fnMesh.setPoints(this->positions, MSpace::kObject);
	CHECK_MSTATUS_AND_RETURN_IT(stat);

	return MS::kSuccess;
}
