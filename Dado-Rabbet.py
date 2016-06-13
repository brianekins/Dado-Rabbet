#Author-Brian Ekins
#Description-

import adsk.core, adsk.fusion, adsk.cam, traceback
import math

_app = adsk.core.Application.cast(None)
_ui = adsk.core.UserInterface.cast(None)

# Global variable used to maintain a reference to all event handlers.
handlers = []


def CreateDado(face1, edge1, width, depth, offset):
    try:
        face = adsk.fusion.BRepFace.cast(face1)
        edge = adsk.fusion.BRepEdge.cast(edge1)
        
        # Get the opposing edge.
        outerLoop = None
        loop = adsk.fusion.BRepLoop.cast(None)
        for loop in face.loops:
            if loop.isOuter:
                outerLoop = loop
                break
        
        for edgeIndex in range(0,4):
            if edge == outerLoop.edges.item(edgeIndex):
                break
    
        if edgeIndex == 0:
            oppositeEdge = outerLoop.edges.item(2)
            otherEdge1 = outerLoop.edges.item(1)
            otherEdge2 = outerLoop.edges.item(3)
        elif edgeIndex == 1:
            oppositeEdge = outerLoop.edges.item(3)
            otherEdge1 = outerLoop.edges.item(0)
            otherEdge2 = outerLoop.edges.item(2)
        elif edgeIndex == 2:
            oppositeEdge = outerLoop.edges.item(0)
            otherEdge1 = outerLoop.edges.item(1)
            otherEdge2 = outerLoop.edges.item(3)
        elif edgeIndex == 3:
            oppositeEdge = outerLoop.edges.item(1)
            otherEdge1 = outerLoop.edges.item(0)
            otherEdge2 = outerLoop.edges.item(2)
    
        comp = face.body.parentComponent
        
        # Create a sketch on the face.
        sk = comp.sketches.add(face)
        
        skLines = adsk.fusion.SketchLines.cast(sk.sketchCurves.sketchLines)
        for i in range(0,4):
            skLines.item(0).deleteMe()        

        skLine = sk.include(edge).item(0)
        skOppositeLine = sk.include(oppositeEdge).item(0)
        skOtherLine1 = sk.include(otherEdge1).item(0)       
        skOtherLine2 = sk.include(otherEdge2).item(0)        
                
        mid1 = adsk.core.Point3D.create((skLine.startSketchPoint.geometry.x + skLine.endSketchPoint.geometry.x)/2, (skLine.startSketchPoint.geometry.y + skLine.endSketchPoint.geometry.y)/2, 0)
        mid2 = adsk.core.Point3D.create((skOppositeLine.startSketchPoint.geometry.x + skOppositeLine.endSketchPoint.geometry.x)/2, (skOppositeLine.startSketchPoint.geometry.y + skOppositeLine.endSketchPoint.geometry.y)/2, 0)
        offsetDir = mid1.vectorTo(mid2)
        lineDir = skLine.startSketchPoint.geometry.vectorTo(skLine.endSketchPoint.geometry)
        lineDir.normalize()
        lineDir.scaleBy(skLine.length)
        
        tempVec = offsetDir.copy()
        tempVec.normalize()
        tempVec.scaleBy(offset)
        midStart = mid1.copy()
        midStart.translateBy(tempVec)
        
        pnt1 = midStart.copy()
        tempVec = lineDir.copy()
        tempVec.scaleBy(-0.5)
        pnt1.translateBy(tempVec)
        
        pnt2 = pnt1.copy()
        pnt2.translateBy(lineDir)
        
        pnt3 = pnt2.copy()
        tempVec = offsetDir.copy()
        tempVec.normalize()
        tempVec.scaleBy(width)
        pnt3.translateBy(tempVec)
    
        pnt4 = pnt3.copy()
        tempVec = lineDir.copy()
        tempVec.scaleBy(-1)
        pnt4.translateBy(tempVec)
    
        line1 = skLines.addByTwoPoints(pnt1, pnt2) 
        line2 = skLines.addByTwoPoints(line1.endSketchPoint, pnt3)
        line3 = skLines.addByTwoPoints(line2.endSketchPoint, pnt4)
        line4 = skLines.addByTwoPoints(line3.endSketchPoint, line1.startSketchPoint)
        
        sk.geometricConstraints.addPerpendicular(line1, line2)
        sk.geometricConstraints.addPerpendicular(line2, line3)
        sk.geometricConstraints.addPerpendicular(line3, line4)
        sk.geometricConstraints.addCoincident(line1.startSketchPoint, skOtherLine1)
        sk.geometricConstraints.addCoincident(line1.endSketchPoint, skOtherLine2)
        sk.geometricConstraints.addParallel(skLine, line1)
        
        if offset == 0:
            sk.geometricConstraints.addCoincident(line1.startSketchPoint, skLine)
        
        if offset != 0:
            dimPoint = mid1.copy()
            offsetDir.normalize()
            offsetDir.scaleBy(offset/2)
            dimPoint.translateBy(offsetDir)
            offsetDim = sk.sketchDimensions.addOffsetDimension(skLine, line1, dimPoint)
        
        dimPoint = mid1.copy()
        offsetDir.normalize()
        offsetDir.scaleBy(offset + (width/2))
        dimPoint.translateBy(offsetDir)        
        offsetDim = sk.sketchDimensions.addOffsetDimension(line1, line3, dimPoint)
        
        prof = adsk.fusion.Profile.cast(None)
        goodProf = None
        for prof in sk.profiles:
            areaProps = prof.areaProperties()
            if WithinTol(areaProps.area, line1.length * line2.length, 0.0001):
                goodProf = prof
                break
            
        extInput = comp.features.extrudeFeatures.createInput(goodProf, adsk.fusion.FeatureOperations.CutFeatureOperation)
        extInput.setDistanceExtent(False, adsk.core.ValueInput.createByReal(-depth))
        ext = comp.features.extrudeFeatures.add(extInput)
        
        des = comp.parentDesign
        tlNode = des.timeline.timelineGroups.add(sk.timelineObject.index, ext.timelineObject.index)
        
        if offset == 0:
            tlNode.name = 'Dado'
        else:
            tlNode.name = 'Rabbet'
    except:
        if _ui:
            _ui.messageBox('Failed:\n{}'.format(traceback.format_exc())) 


def WithinTol(value1, value2, tolerance):
    if math.fabs(value1 - value2) <= tolerance:
        return True
    else:
        return False
        

class DadoRabbetCommandChangedHandler(adsk.core.InputChangedEventHandler):
    def __init__(self):
        super().__init__()
    def notify(self, args):
        eventArgs = adsk.core.InputChangedEventArgs.cast(args)

        if eventArgs.input.id == 'jointType':
            jointType = adsk.core.DropDownCommandInput.cast(eventArgs.input)
            offsetInput = eventArgs.inputs.itemById('offsetInput')
            if jointType.selectedItem.name == 'Dado':
                offsetInput.isVisible = True
            elif jointType.selectedItem.name == 'Rabbet':
                offsetInput.isVisible = False
        elif eventArgs.input.id == 'faceSelect':
            eventArgs.inputs.itemById('edgeSelect').isEnabled = True


# Event handler for the selectionEvent event.
class DadoRabbetCommandSelectionEventHandler(adsk.core.SelectionEventHandler):
    def __init__(self):
        super().__init__()
    def notify(self, args):
        eventArgs = adsk.core.SelectionEventArgs.cast(args)
        
        if eventArgs.activeInput.id == 'faceSelect':
            face = eventArgs.selection.entity
            if isValidFace(face):
                eventArgs.isSelectable = True
            else:
                eventArgs.isSelectable = False
        elif eventArgs.activeInput.id == 'edgeSelect':
            inputs = adsk.core.CommandInputs.cast(eventArgs.firingEvent.sender.commandInputs)
            faceSelect = adsk.core.SelectionCommandInput.cast(inputs.itemById('faceSelect'))
            if faceSelect.selectionCount == 1:
                face = faceSelect.selection(0).entity
                edge = eventArgs.selection.entity
                if isValidEdge(face, edge):
                    eventArgs.isSelectable = True
                else:
                    eventArgs.isSelectable = False
            else:
                eventArgs.isSelectable = False


def isValidEdge(face, edge):
    if edge.geometry.curveType != adsk.core.Curve3DTypes.Line3DCurveType:
        return False
        
    if edge.faces.item(0) == face:
        return True
    elif edge.faces.item(1) == face:
        return True
        
    return False
       

def isValidFace(face):
    outerLoop = adsk.fusion.BRepLoop.cast(None)
    loop = adsk.fusion.BRepLoop.cast(None)
    for loop in face.loops:
        if loop.isOuter:
            outerLoop = loop
            break

    if outerLoop.edges.count != 4:
        return False
        
    edge = adsk.fusion.BRepEdge.cast(None)
    lineVectors = []
    for edge in loop.edges:
        ln = adsk.core.Line3D.cast(edge.geometry)
        lineVectors.append(ln.startPoint.vectorTo(ln.endPoint))
        if edge.geometry.curveType != adsk.core.Curve3DTypes.Line3DCurveType:
            return False
            
    if not lineVectors[0].isPerpendicularTo(lineVectors[1]):
        return False
    elif not lineVectors[0].isParallelTo(lineVectors[2]):
        return False
    elif not lineVectors[0].isPerpendicularTo(lineVectors[3]):
        return False

    return True


# Event handler for the executePreview event.
class DadoRabbetCommandExecutePreviewHandler(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()
    def notify(self, args):
        eventArgs = adsk.core.CommandEventArgs.cast(args)
        inputs = eventArgs.command.commandInputs

        # Get the values of all of the inputs.
        face = inputs.itemById('faceSelect').selection(0).entity
        edge = inputs.itemById('edgeSelect').selection(0).entity
        width = inputs.itemById('widthInput').value
        depth = inputs.itemById('depthInput').value
        if inputs.itemById('jointType').selectedItem.name == 'Dado':
            offset = inputs.itemById('offsetInput').value
        else:
            offset = 0
            
        CreateDado(face, edge, width, depth, offset)        

        eventArgs.isValidResult = True


# Event handler for the execute event.
class DadoRabbetCommandExecuteHandler(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()
    def notify(self, args):
        eventArgs = adsk.core.CommandEventArgs.cast(args)
        inputs = eventArgs.command.commandInputs

        # Get the values of all of the inputs.
        face = inputs.itemById('faceSelect').selection(0).entity
        edge = inputs.itemById('edgeSelect').selection(0).entity
        width = inputs.itemById('widthInput').value
        depth = inputs.itemById('depthInput').value
        if inputs.itemById('jointType').selectedItem.name == 'Dado':
            offset = inputs.itemById('offsetInput').value
        else:
            offset = 0
            
        CreateDado(face, edge, width, depth, offset)        


# Event handler for the commandCreated event.
class DadoRabbetCommandCreatedHandler(adsk.core.CommandCreatedEventHandler):
    def __init__(self):
        super().__init__()
    def notify(self, args):
        try:
            eventArgs = adsk.core.CommandCreatedEventArgs.cast(args)
    
            # Create the various command inputs.
            cmd = eventArgs.command
            inputs = cmd.commandInputs
            
            jointTypeDropDown = inputs.addDropDownCommandInput('jointType', 'Joint type', adsk.core.DropDownStyles.LabeledIconDropDownStyle)
            jointTypeDropDown.listItems.add('Dado', True, 'Resources/DadoType')
            jointTypeDropDown.listItems.add('Rabbet', False, 'Resources/RabbetType')
    
            selectInput = inputs.addSelectionInput('faceSelect', 'Face', 'Select the face the joint will be cut into')
            selectInput.addSelectionFilter('PlanarFaces')
            selectInput.setSelectionLimits(1,1)
    
            selectInput = inputs.addSelectionInput('edgeSelect', 'Edge', 'Select the edge the joint will be positioned from.')
            selectInput.addSelectionFilter('LinearEdges')
            selectInput.setSelectionLimits(1,1)
    
            des = adsk.fusion.Design.cast(_app.activeProduct)
            uom = des.unitsManager
            inputs.addValueInput('widthInput', 'Width', uom.defaultLengthUnits, adsk.core.ValueInput.createByReal(2.54 * .75))
            inputs.addValueInput('depthInput', 'Depth', uom.defaultLengthUnits, adsk.core.ValueInput.createByReal(2.54 * .375))
            inputs.addValueInput('offsetInput', 'Offset', uom.defaultLengthUnits, adsk.core.ValueInput.createByReal(2.54 * 12))
    
            # Connect to the command events.
            onInputChanged = DadoRabbetCommandChangedHandler()
            cmd.inputChanged.add(onInputChanged)
            handlers.append(onInputChanged)
            
            onSelectionEvent = DadoRabbetCommandSelectionEventHandler()
            cmd.selectionEvent.add(onSelectionEvent)
            handlers.append(onSelectionEvent)
            
            onExecute = DadoRabbetCommandExecuteHandler()
            cmd.execute.add(onExecute)
            handlers.append(onExecute)
            
            onExecutePreview = DadoRabbetCommandExecutePreviewHandler()
            cmd.executePreview.add(onExecutePreview)
            handlers.append(onExecutePreview)
        except:
            if _ui:
                _ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
 
                
def run(context):
    try:
        global _app, _ui
        _app = adsk.core.Application.get()
        _ui  = _app.userInterface
        
        # Add a button to the user interface.
        cmdDef = _ui.commandDefinitions.addButtonDefinition('ekinsDadoRabbet', 'Dado/Rabbet', 'Create a dado or rabbet joint', 'Resources/DadoRabbet')
        modifyPanel = _ui.allToolbarPanels.itemById('SolidModifyPanel')
        cntrl = modifyPanel.controls.addCommand(cmdDef)
        
        # Connect the command created event.
        onCommandCreated = DadoRabbetCommandCreatedHandler()
        cmdDef.commandCreated.add(onCommandCreated)
        handlers.append(onCommandCreated)        
    except:
        if _ui:
            _ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))

def stop(context):
    try:
        cmdDef = _ui.commandDefinitions.itemById('ekinsDadoRabbet')
        if cmdDef:
            cmdDef.deleteMe()
            
        modifyPanel = _ui.allToolbarPanels.itemById('SolidModifyPanel')
        cntrl = modifyPanel.controls.itemById('ekinsDadoRabbet')
        if cntrl:
            cntrl.deleteMe()
        

    except:
        if _ui:
            _ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
