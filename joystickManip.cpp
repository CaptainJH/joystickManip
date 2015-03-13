//-
// ==========================================================================
// Copyright 1995,2006,2008 Autodesk, Inc. All rights reserved.
//
// Use of this software is subject to the terms of the Autodesk
// license agreement provided at the time of installation or download,
// or which otherwise accompanies this software in either electronic
// or hard copy form.
// ==========================================================================
//+

/* 

	This is an example to demonstrate the use of a rotation manipulator through
	a rotation tool and context.  This example uses three classes to accomplish
	this task: First, a context command (joystickContext) is provided to create
	instances of the context.  Next, a custom selection context
	(joystickManipContext) is created to manage the rotation manipulator.  
	Finally, the rotation manipulator is provided as a custom node class.
	
	Loading and unloading:
	----------------------

	The rotate manipulator context can be created with the 
	following mel commands:
    
		joystickContext;
		setToolTo joystickContext1;

	If the preceding commands were used to create the manipulator context, 
	the following commands can destroy it:

		deleteUI joystickContext1;
		deleteUI rotateManip;

	If the plugin is loaded and unloaded frequently (eg. during testing),
	it is useful to make these command sequences into shelf buttons.

	How to use:
	-----------

	Once the tool button has been created using the script above, select the
	tool button then click on an object.  The rotate manipulator should appear
	at the center of the selected object.  The rotate manipulator can be used
	much like the built-in rotate manipulator.  In addition, the plugin 
	produces a state manipulator that can be used to control the modes for the
	rotation manipulator.  The state manipulator should be displayed 2 units 
	along the X-axis from the object.
	
*/

#include <maya/MIOStream.h>
#include <stdio.h>
#include <stdlib.h>

#include <maya/MFn.h>
#include <maya/MPxNode.h>
#include <maya/MPxManipContainer.h>
#include <maya/MPxSelectionContext.h>
#include <maya/MPxContextCommand.h>
#include <maya/MModelMessage.h>
#include <maya/MFnPlugin.h>
#include <maya/MGlobal.h>
#include <maya/MItSelectionList.h>
#include <maya/MPoint.h>
#include <maya/MVector.h>
#include <maya/MDagPath.h>
#include <maya/MManipData.h>
#include <maya/MEulerRotation.h>
#include <maya/MFnNumericAttribute.h>

#include <maya/MDGModifier.h>
#include <maya/MPlug.h>

#include <maya/MDGMessage.h>

// Manipulators
#include <maya/MFnRotateManip.h>
#include <maya/MFnStateManip.h>

#include <math.h>
#include <map>
#include <SFML/Window/Joystick.hpp>

const float DeadZone = 3.0f;

class XboxControllerToMaya
{
public:
	XboxControllerToMaya(unsigned int id, float deadzone=1.0f)
		: joystickID_(id)
		, deadzone_(deadzone)
	{
		sf::Joystick::update();
	}

	enum ButtonName
	{
		X, Y, A, B, RB, LB, R3, L3, BACK, START
	};

	enum AxisName
	{
		Left, Right, Trigger, DPad
	};

	typedef void (*ButtonCallback)();
	typedef void (*AxisCallback)(float x, float y);

	void update();
	bool registerButtonCallback(ButtonName btn, ButtonCallback callback, bool forceWrite=false);
	bool registerAxisCallback(AxisName axis, AxisCallback callback, bool forceWrite=false);

protected:
	unsigned int joystickID_;
	const float deadzone_;

	std::map<ButtonName, ButtonCallback> btnCallbackTable_;
	std::map<AxisName, AxisCallback> axisCallbackTable_;

	float deadZoneFilter(float in_) const
	{
		return abs(in_) < deadzone_ ? 0.0f : in_;
	}

	static int ButtonNameToSFML(ButtonName btn);
};

int XboxControllerToMaya::ButtonNameToSFML(XboxControllerToMaya::ButtonName btn)
{
	int result = 0;
	switch(btn)
	{
	case XboxControllerToMaya::X:
		result = 3;
		break;

	case XboxControllerToMaya::Y:
		result = 4;
		break;

	case XboxControllerToMaya::A:
		result = 1;
		break;

	case XboxControllerToMaya::B:
		result = 2;
		break;

	case XboxControllerToMaya::RB:
		result = 6;
		break;

	case XboxControllerToMaya::LB:
		result = 5;
		break;

	case XboxControllerToMaya::BACK:
		result = 7;
		break;

	case XboxControllerToMaya::START:
		result = 8;
		break;

	case XboxControllerToMaya::R3:
		result = 10;
		break;

	case XboxControllerToMaya::L3:
		result = 9;
		break;
	};

	return result - 1;
}

void XboxControllerToMaya::update()
{
	sf::Joystick::update();

	for(auto it = btnCallbackTable_.begin(); it != btnCallbackTable_.end(); ++it)
	{
		ButtonName btn = it->first;
		if(sf::Joystick::isButtonPressed(joystickID_, ButtonNameToSFML(btn)))
		{
			(it->second)();
		}
	}

	for(auto it = axisCallbackTable_.begin(); it != axisCallbackTable_.end(); ++it)
	{
		AxisName axis = it->first;
		
		if(axis == XboxControllerToMaya::Left)
		{
			auto x = sf::Joystick::getAxisPosition(joystickID_, sf::Joystick::X);
			auto y = -sf::Joystick::getAxisPosition(joystickID_, sf::Joystick::Y);
			(it->second)(deadZoneFilter(x), deadZoneFilter(y));
		}
		else if(axis == XboxControllerToMaya::Right)
		{
			auto x = sf::Joystick::getAxisPosition(joystickID_, sf::Joystick::U);
			auto y = -sf::Joystick::getAxisPosition(joystickID_, sf::Joystick::R);
			(it->second)(deadZoneFilter(x), deadZoneFilter(y));
		}
		else if(axis == XboxControllerToMaya::Trigger)
		{
			auto x = sf::Joystick::getAxisPosition(joystickID_, sf::Joystick::Z);
			if(x > 0)
				(it->second)(deadZoneFilter(x), 0.0f);
			else
				(it->second)(0.0f, -deadZoneFilter(x));
		}
		else if(axis == XboxControllerToMaya::DPad)
		{
			auto x = sf::Joystick::getAxisPosition(joystickID_, sf::Joystick::PovX);
			auto y = sf::Joystick::getAxisPosition(joystickID_, sf::Joystick::PovY);
			(it->second)(deadZoneFilter(x), deadZoneFilter(y));
		}
	}
}

bool XboxControllerToMaya::registerButtonCallback(XboxControllerToMaya::ButtonName btn, XboxControllerToMaya::ButtonCallback callback, bool forceWrite/*=false*/)
{
	if(btnCallbackTable_.find(btn) != btnCallbackTable_.end() && !forceWrite)
		return false;

	btnCallbackTable_[btn] = callback;
	return true;
}

bool XboxControllerToMaya::registerAxisCallback(XboxControllerToMaya::AxisName axis, XboxControllerToMaya::AxisCallback callback, bool forceWrite/*=false*/)
{
	if(axisCallbackTable_.find(axis) != axisCallbackTable_.end() && !forceWrite)
		return false;

	axisCallbackTable_[axis] = callback;
	return true;
}


// This function is a utility that can be used to extract vector values from
// plugs.
//
MVector vectorPlugValue(const MPlug& plug) 
{
	if (plug.numChildren() == 3)
	{
		double x,y,z;
		MPlug rx = plug.child(0);
		MPlug ry = plug.child(1);
		MPlug rz = plug.child(2);
		rx.getValue(x);
		ry.getValue(y);
		rz.getValue(z);
		MVector result(x,y,z);
		return result;
	}
	else {
		MGlobal::displayError("Expected 3 children for plug "+MString(plug.name()));
		MVector result(0,0,0);
		return result;
	}
}

/////////////////////////////////////////////////////////////
//
// joystickManip
//
// This class implements the example rotate manipulator.
//
/////////////////////////////////////////////////////////////

class joystickManip : public MPxManipContainer
{
public:
	joystickManip();
	virtual ~joystickManip();
	
	static void * creator();
	static MStatus initialize();
	virtual MStatus createChildren();
	virtual MStatus connectToDependNode(const MObject &node);

	virtual void draw(M3dView &view, 
					  const MDagPath &path, 
					  M3dView::DisplayStyle style,
					  M3dView::DisplayStatus status);

	MVector nodeTranslation() const;

	// Callback function
	MManipData rotationChangedCallback(unsigned index);
	static void timeChangeCallback( MTime& time, void* data );

	static void leftCallback(float x, float y);
	static void rightCallback(float x, float y);
	static void triggerCallback(float x ,float y);
	static void l3Callback();

public:
	static MTypeId id;
	static XboxControllerToMaya* joystick_;
	static const MObject *bindingNode_;
	static MObject		nodeTranslation_;

private:
	MDagPath fRotateManip;
	MDagPath fStateManip;

	MDagPath fNodePath;

	unsigned rotatePlugIndex;

	MCallbackId timechangeCallback_;

};

MTypeId joystickManip::id( 0x80022 );
XboxControllerToMaya* joystickManip::joystick_ = 0;
const MObject* joystickManip::bindingNode_ = 0;
MObject joystickManip::nodeTranslation_;

joystickManip::joystickManip() 
{ 
	// The constructor must not call createChildren for user-defined
	// manipulators.
	timechangeCallback_ = MDGMessage::addTimeChangeCallback(&timeChangeCallback, (void*) this);
	if(joystick_)
	{
		joystick_->registerAxisCallback(XboxControllerToMaya::Left, joystickManip::leftCallback);
		joystick_->registerAxisCallback(XboxControllerToMaya::Right, joystickManip::rightCallback);
		joystick_->registerAxisCallback(XboxControllerToMaya::Trigger, joystickManip::triggerCallback);
		joystick_->registerButtonCallback(XboxControllerToMaya::L3, joystickManip::l3Callback);
	}
}

joystickManip::~joystickManip() 
{
}


void *joystickManip::creator()
{
	joystick_ = new XboxControllerToMaya(0);
	return new joystickManip();
}


MStatus joystickManip::initialize()
{
	return MPxManipContainer::initialize();
}


MStatus joystickManip::createChildren()
{
	MStatus stat = MStatus::kSuccess;

	// Add the rotation manip
	//
	fRotateManip = addRotateManip("RotateManip", "rotation");

	// Add the state manip.  The state manip is used to cycle through the 
	// rotate manipulator modes to demonstrate how they work.
	//
	fStateManip = addStateManip("StateManip", "state");

	// The state manip permits 4 states.  These correspond to:
	// 0 - Rotate manip in objectSpace mode
	// 1 - Rotate manip in worldSpace mode
	// 2 - Rotate manip in gimbal mode
	// 3 - Rotate manip in objectSpace mode with snapping on
	//
	// Note that while the objectSpace and gimbal modes will operator similar 
	// to the built-in Maya rotate manipulator, the worldSpace mode will 
	// produce unusual rotations because the plugin does not convert worldSpace
	// rotations to object space.
	//
	MFnStateManip stateManip(fStateManip);
	stateManip.setMaxStates(4);
	stateManip.setInitialState(0);
	
	return stat;
}


MStatus joystickManip::connectToDependNode(const MObject &node)
{
	MGlobal::displayError("joystickManip::connectToDependNode");

	MStatus stat;

	// Find the rotate and rotatePivot plugs on the node.  These plugs will 
	// be attached either directly or indirectly to the manip values on the
	// rotate manip.
	//
	MFnDependencyNode nodeFn(node);
	MPlug rPlug = nodeFn.findPlug("rotate", &stat);
	if (!stat)
	{
		MGlobal::displayError("Could not find rotate plug on node");
		return stat;
	}
	MPlug rcPlug = nodeFn.findPlug("rotatePivot", &stat);
	if (!stat)
	{
		MGlobal::displayError("Could not find rotatePivot plug on node");
		return stat;
	}

	// If the translate pivot exists, it will be used to move the state manip
	// to a convenient location.
	//
	MPlug tPlug = nodeFn.findPlug("translate", &stat);

	// To avoid having the object jump back to the default rotation when the
	// manipulator is first used, extract the existing rotation from the node
	// and set it as the initial rotation on the manipulator.
	//
	MEulerRotation existingRotation(vectorPlugValue(rPlug));
	MVector existingTranslation(vectorPlugValue(tPlug));

	// 
	// The following code configures default settings for the rotate 
	// manipulator.
	//

	MFnRotateManip rotateManip(fRotateManip);
	rotateManip.setInitialRotation(existingRotation);
	rotateManip.setRotateMode(MFnRotateManip::kObjectSpace);
	rotateManip.displayWithNode(node);

	// Add a callback function to be called when the rotation value changes
	//
	rotatePlugIndex = addManipToPlugConversionCallback( rPlug, 
		(manipToPlugConversionCallback)
		&joystickManip::rotationChangedCallback );

	// Create a direct (1-1) connection to the rotation center plug
	//
	rotateManip.connectToRotationCenterPlug(rcPlug);

	// Place the state manip at a distance of 2.0 units away from the object
	// along the X-axis.
	//
	MFnStateManip stateManip(fStateManip);
	stateManip.setTranslation(existingTranslation+MVector(2,0,0),
		MSpace::kTransform);

	// add the rotate XYZ plugs to the In-View Editor
	//
	MPlug rxPlug = rPlug.child( 0 );
	addPlugToInViewEditor( rxPlug );
	MPlug ryPlug = rPlug.child( 1 );
	addPlugToInViewEditor( ryPlug );
	MPlug rzPlug = rPlug.child( 2 );
	addPlugToInViewEditor( rzPlug );

	finishAddingManips();
	MPxManipContainer::connectToDependNode(node);
	return stat;
}


void joystickManip::draw(M3dView & view, 
					 const MDagPath & path, 
					 M3dView::DisplayStyle style,
					 M3dView::DisplayStatus status)
{
	// Uses default manipulator drawing to draw the rotate and state manips
	//
	MPxManipContainer::draw(view, path, style, status);

}

MManipData joystickManip::rotationChangedCallback(unsigned index) {
	static MEulerRotation cache;
	MObject obj = MObject::kNullObj;

	// If we entered the callback with an invalid index, print an error and
	// return.  Since we registered the callback only for one plug, all 
	// invocations of the callback should be for that plug.
	//
	if (index != rotatePlugIndex)
	{
		MGlobal::displayError("Invalid index in rotation changed callback!");

		// For invalid indices, return vector of 0's
		MFnNumericData numericData;
		obj = numericData.create( MFnNumericData::k3Double );
		numericData.setData(0.0,0.0,0.0);

		return obj;
	}

	// Assign function sets to the manipulators
	//
	MFnStateManip stateManip(fStateManip);
	MFnRotateManip rotateManip(fRotateManip);

	// Adjust settings on the rotate manip based on the state of the state 
	// manip.
	//
	int mode = stateManip.state();
	if (mode != 3)
	{
		rotateManip.setRotateMode((MFnRotateManip::RotateMode) stateManip.state());
		rotateManip.setSnapMode(false);
	}
	else {
		// State 3 enables snapping for an object space manip.  In this case,
		// we snap every 15.0 degrees.
		//
		rotateManip.setRotateMode(MFnRotateManip::kObjectSpace);
		rotateManip.setSnapMode(true);
		rotateManip.setSnapIncrement(15.0);
	}

	// The following code creates a data object to be returned in the 
	// MManipData.  In this case, the plug to be computed must be a 3-component
	// vector, so create data as MFnNumericData::k3Double
	//
	MFnNumericData numericData;
	obj = numericData.create( MFnNumericData::k3Double );

	// Retrieve the value for the rotation from the manipulator and return it
	// directly without modification.  If the manipulator should eg. slow down
	// rotation, this method would need to do some math with the value before
	// returning it.
	//
	MEulerRotation manipRotation;
	if (!getConverterManipValue (rotateManip.rotationIndex(), manipRotation))
	{
		MGlobal::displayError("Error retrieving manip value");
		numericData.setData(0.0,0.0,0.0);
	}
	else {
		numericData.setData(manipRotation.x, manipRotation.y, manipRotation.z);
	}

	return MManipData(obj);
}

void joystickManip::timeChangeCallback( MTime& time, void* data )
{
	joystick_->update();
}


void joystickManip::l3Callback()
{
	MSelectionList list;
	MGlobal::getActiveSelectionList( list );

	for ( MItSelectionList iter( list ); !iter.isDone(); iter.next() ) 
	{
		MObject node;
		MStatus status;
		iter.getDependNode( node );
		MFnTransform xform( node, &status );
		if ( MS::kSuccess == status )
		{
			xform.setTranslation(MVector(0.0, 0.0, 0.0), MSpace::kObject);
		}
	}
}

void joystickManip::triggerCallback(float x, float y)
{
	MSelectionList list;
	MGlobal::getActiveSelectionList( list );


	for ( MItSelectionList iter( list ); !iter.isDone(); iter.next() ) 
	{
		MObject node;
		MStatus status;
		iter.getDependNode( node );
		MFnTransform xform( node, &status );
		if ( MS::kSuccess == status )
		{
			double newScale[3];

			if(abs(x) > DeadZone)
			{
				newScale[0] = x / 10;
				newScale[1] = x / 10;
				newScale[2] = x / 10;
			}
			else if(abs(y) > DeadZone)
			{
				newScale[0] = 10 / y;
				newScale[1] = 10 / y;
				newScale[2] = 10 / y;
			}
			else
			{
				newScale[0] = 1.0;
				newScale[1] = 1.0;
				newScale[2] = 1.0;

			}

			xform.setScale( newScale );
		}
	}
}

void joystickManip::rightCallback(float x, float y)
{
	MSelectionList list;
	MGlobal::getActiveSelectionList( list );

	if(abs(x) > DeadZone || abs(y) > DeadZone)
	{
		for ( MItSelectionList iter( list ); !iter.isDone(); iter.next() ) 
		{
			MObject node;
			MStatus status;
			iter.getDependNode( node );
			MFnTransform xform( node, &status );
			if ( MS::kSuccess == status )
			{
				MEulerRotation rot(0.0, x / 100, 0.0);
				xform.rotateBy(rot);
			}
		}
		
	}
}

void joystickManip::leftCallback(float x, float y)
{
	MSelectionList list;
	MGlobal::getActiveSelectionList( list );

	if(abs(x) > DeadZone || abs(y) > DeadZone)
	{
		for ( MItSelectionList iter( list ); !iter.isDone(); iter.next() ) 
		{
			MObject node;
			MStatus status;
			iter.getDependNode( node );
			MFnTransform xform( node, &status );
			if ( MS::kSuccess == status )
			{
				xform.translateBy(MVector(x / 100, 0.0, y / 100), MSpace::kPostTransform);
			}
		}
		
	}
	//else
	//{
	//	for ( MItSelectionList iter( list ); !iter.isDone(); iter.next() ) 
	//	{
	//		MObject node;
	//		MStatus status;
	//		iter.getDependNode( node );
	//		MFnTransform xform( node, &status );
	//		if ( MS::kSuccess == status )
	//		{
	//			xform.setTranslation(MVector(0.0, 0.0, 0.0), MSpace::kObject);
	//		}
	//	}
	//}
}

// Return the node translation
MVector joystickManip::nodeTranslation() const
{
	MFnDagNode dagFn(fNodePath);
	MDagPath path;
	dagFn.getPath(path);
	path.pop();  // pop from the shape to the transform
	MFnTransform transformFn(path);
	return transformFn.translation(MSpace::kWorld);
}

/////////////////////////////////////////////////////////////
//
// joystickManipContext
//
// This class is a simple context for supporting a rotate manipulator.
//
/////////////////////////////////////////////////////////////

class joystickManipContext : public MPxSelectionContext
{
public:
	joystickManipContext();
	virtual void	toolOnSetup(MEvent &event);
	virtual void	toolOffCleanup();

	// Callback issued when selection list changes
	static void updateManipulators(void * data);

private:
	MCallbackId id1;
};

joystickManipContext::joystickManipContext()
{
	MString str("Plugin Rotate Manipulator");
	setTitleString(str);
}


void joystickManipContext::toolOnSetup(MEvent &)
{
	MString str("Rotate the object using the rotation handles");
	setHelpString(str);

	updateManipulators(this);
	MStatus status;
	id1 = MModelMessage::addCallback(MModelMessage::kActiveListModified,
									 updateManipulators, 
									 this, &status);
	if (!status) {
		MGlobal::displayError("Model addCallback failed");
	}
}


void joystickManipContext::toolOffCleanup()
{
	MStatus status;
	status = MModelMessage::removeCallback(id1);
	if (!status) {
		MGlobal::displayError("Model remove callback failed");
	}
	MPxContext::toolOffCleanup();
}


void joystickManipContext::updateManipulators(void * data)
{
	MStatus stat = MStatus::kSuccess;
	
	joystickManipContext * ctxPtr = (joystickManipContext *) data;
	ctxPtr->deleteManipulators(); 

	// Add the rotate manipulator to each selected object.  This produces 
	// behavior different from the default rotate manipulator behavior.  Here,
	// a distinct rotate manipulator is attached to every object.
	// 
	MSelectionList list;
	stat = MGlobal::getActiveSelectionList(list);
	MItSelectionList iter(list, MFn::kInvalid, &stat);

	if (MS::kSuccess == stat) {
		for (; !iter.isDone(); iter.next()) {

			// Make sure the selection list item is a depend node and has the
			// required plugs before manipulating it.
			//
			MObject dependNode;
			iter.getDependNode(dependNode);
			if (dependNode.isNull() || !dependNode.hasFn(MFn::kDependencyNode))
			{
				MGlobal::displayWarning("Object in selection list is not "
					"a depend node.");
				continue;
			}

			MFnDependencyNode dependNodeFn(dependNode);
			/* MPlug rPlug = */ dependNodeFn.findPlug("rotate", &stat);
			if (!stat) {
				MGlobal::displayWarning("Object cannot be manipulated: " +
					dependNodeFn.name());
				continue;
			}

			// Add manipulator to the selected object
			//
			MString manipName ("joystickManip");
			MObject manipObject;
			joystickManip* manipulator =
				(joystickManip *) joystickManip::newManipulator(
					manipName, 
					manipObject);

			if (NULL != manipulator) {
				// Add the manipulator
				//
				ctxPtr->addManipulator(manipObject);

				// Connect the manipulator to the object in the selection list.
				//
				if (!manipulator->connectToDependNode(dependNode))
				{
					MGlobal::displayWarning("Error connecting manipulator to"
						" object: " + dependNodeFn.name());
				}
			} 
		}
	}
}


/////////////////////////////////////////////////////////////
//
// joystickContext
//
// This is the command that will be used to create instances
// of our context.
//
/////////////////////////////////////////////////////////////

class joystickContext : public MPxContextCommand
{
public:
	joystickContext() {};
	virtual MPxContext * makeObj();

public:
	static void* creator();
};

MPxContext *joystickContext::makeObj()
{
	return new joystickManipContext();
}

void *joystickContext::creator()
{
	return new joystickContext;
}


///////////////////////////////////////////////////////////////////////
//
// The following routines are used to register/unregister
// the context and manipulator
//
///////////////////////////////////////////////////////////////////////

MStatus initializePlugin(MObject obj)
{
	MStatus status;
	MFnPlugin plugin(obj, PLUGIN_COMPANY, "6.0", "Any");

	status = plugin.registerContextCommand("joystickContext",
										   &joystickContext::creator);
	if (!status) {
		MGlobal::displayError("Error registering joystickContext command");
		return status;
	}

	status = plugin.registerNode("joystickManip", joystickManip::id, 
								 &joystickManip::creator, &joystickManip::initialize,
								 MPxNode::kManipContainer);
	if (!status) {
		MGlobal::displayError("Error registering rotateManip node");
		return status;
	}

	return status;
}


MStatus uninitializePlugin(MObject obj)
{
	MStatus status;
	MFnPlugin plugin(obj);

	status = plugin.deregisterContextCommand("joystickContext");
	if (!status) {
		MGlobal::displayError("Error deregistering joystickContext command");
		return status;
	}

	status = plugin.deregisterNode(joystickManip::id);
	if (!status) {
		MGlobal::displayError("Error deregistering RotateManip node");
		return status;
	}

	return status;
}
