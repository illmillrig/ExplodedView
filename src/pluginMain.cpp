
#include "ExplodedView.h"
#include <maya/MFnPlugin.h>


MStatus initializePlugin( MObject obj ) {
	MStatus   status;
	MFnPlugin plugin( obj, "Travis Miller", "2016", "Any");

	status = plugin.registerNode( "ExplodedView", ExplodedView::id, ExplodedView::creator,
								  ExplodedView::initialize, MPxNode::kDeformerNode );
	if (!status) {
		status.perror("registerNode");
		return status;
	}

	return status;
}

MStatus uninitializePlugin( MObject obj) {
	MStatus   status;
	MFnPlugin plugin( obj );

	status = plugin.deregisterNode( ExplodedView::id );
	if (!status) {
		status.perror("deregisterNode");
		return status;
	}

	return status;
}
