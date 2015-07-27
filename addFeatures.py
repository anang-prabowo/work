'''
Features addition for ifw biped rig
'''

import pymel.core as pm
import logging

logger = logging.getLogger(__name__)

def addLockKneeElbow():
    '''
    add lock knee/elbow feature
    '''
    ikCtrls = {'FL':('IK_ARM_FL_CTRL', 'ElbowLock', 'PVC_ARM_FL_CTRL' , 'jIKArmFL', 'jIKClavL02'),
               'FR':('IK_ARM_FR_CTRL', 'ElbowLock', 'PVC_ARM_FR_CTRL' , 'jIKArmFR', 'jIKClavR02'),
               'BL':('IK_LEG_BL_CTRL', 'KneeLock', 'PVC_LEG_BL_CTRL', 'jIKLegBL', 'jIKHipBL02'),
               'BR':('IK_LEG_BR_CTRL', 'KneeLock', 'PVC_LEG_BR_CTRL', 'jIKLegBR', 'jIKHipBR02')}
               

    for side in ikCtrls:
        pm.addAttr(ikCtrls[side][0], sn=ikCtrls[side][1], at='bool')
        lockAttr = pm.PyNode('%s.%s'%(ikCtrls[side][0], ikCtrls[side][1]))
        lockAttr.set(0, keyable=True, lock=False)
        
        pt1 = pm.PyNode(ikCtrls[side][4])
        pt2 = pm.PyNode(ikCtrls[side][2])
        pt3 = pm.PyNode(ikCtrls[side][0])
        
        pt1pos = pm.xform(pt1, q=True, ws=True, rp=True)
        pt2pos = pm.xform(pt2, q=True, ws=True, rp=True)
        pt3pos = pm.xform(pt3, q=True, ws=True, rp=True)
        
        dim1 = pm.distanceDimension(startPoint=pt1pos, endPoint=pt2pos)
        dim1Parent = dim1.listRelatives(parent=True)[0]
        dim1Parent.rename('%s_lockPos_01'%side)
        dim2 = pm.distanceDimension(startPoint=pt3pos, endPoint=pt2pos)
        dim2Parent = dim2.listRelatives(parent=True)[0]
        dim2Parent.rename('%s_lockPos_02'%side)
        
        loc = dim1.endPoint.listConnections()[0]
        loc.rename('loc_%s_%s'%(side,ikCtrls[side][1]))
        
        pm.parentConstraint(ikCtrls[side][2], loc, mo=True)
        pm.scaleConstraint('CONSTRAIN', loc, mo=True)
        pm.parent((dim1, dim2, loc), 'misc')
        
        cond = pm.createNode('condition', name='cond_adjust_%s_%s'%(side,ikCtrls[side][1]))
        jnt01 = pm.PyNode('%s02'%ikCtrls[side][3])
        jnt02 = pm.PyNode('%s03'%ikCtrls[side][3])
        
        lockAttr >> cond.firstTerm
        cond.secondTerm.set(1)
        dim1.distance >> cond.colorIfTrueR
        dim2.distance >> cond.colorIfTrueG
        cond.colorIfFalseR.set( jnt01.tx.get() )
        cond.colorIfFalseG.set( jnt02.tx.get() )
        
        cond.outColorR >> jnt01.tx
        cond.outColorG >> jnt02.tx


def fixAutoKneeElbow():
    '''
    Enable\fix auto elbow and auto knee features
    '''
    ikCtrls = {'FL':('attachPVC_ARM_FL_CTRL', 'IK_ARM_FL_CTRL', 'autoElbow'),
               'FR':('attachPVC_ARM_FR_CTRL', 'IK_ARM_FR_CTRL', 'autoElbow'),
               'BL':('attachPVC_LEG_BL_CTRL', 'IK_LEG_BL_CTRL', 'autoKnee'),
               'BR':('attachPVC_LEG_BR_CTRL', 'IK_LEG_BR_CTRL', 'autoKnee')}

    for side in ikCtrls:
        nullParent = pm.listRelatives(ikCtrls[side][0], parent=True)[0]
        null = pm.group(empty=True, name='loc_%s_PVC_lock'%side)

        tempCns = pm.parentConstraint(ikCtrls[side][0], null, mo=False)
        pm.delete(tempCns)
        pm.parent(null, nullParent)
        
        pntConstraint = pm.pointConstraint(null, ikCtrls[side][0], mo=True)
        wtBlend = pntConstraint.listAttr()[-2:]

        condNode = pm.createNode('condition', name='cond_pvcLock_%s'%side)
        condNode.colorIfTrueR.set(0)
        condNode.colorIfTrueG.set(1)
        condNode.colorIfFalseR.set(1)
        condNode.colorIfFalseG.set(0)
        
        ctrlAttr = pm.PyNode('%s.%s'%(ikCtrls[side][1], ikCtrls[side][2]))
        ctrlAttr >> condNode.firstTerm
        
        condNode.outColorR >> wtBlend[0]
        condNode.outColorG >> wtBlend[1]    
        
        ctrlAttr.set(1)


def spineFKIK():
    '''
    Spine FK/IK feature
    '''
    import pymel.core as pm

    pCns = {1:'HIP_CTRL',
            2:'HIP_CTRL',
            3:'SPINE_LOWER_CTRL',
            4:'SPINE_MIDDLE_CTRL',
            5:'SPINE_MIDDLE_CTRL',
            6:'SPINE_UPPER_CTRL'}

    pm.addAttr('WORLD', sn='SpineIKFKSwitch', at='enum', en='IK:FK')
    newAttr = pm.PyNode('WORLD.SpineIKFKSwitch')
    newAttr.set(0, keyable=True, lock=False)

    for i in range(1,7):
        pm.parent('jSpine%02d'%i, 'bindSkelSpine')
        
        cnsNode = pm.parentConstraint(pCns[i], 'jSpine%02d'%i, mo=True)
        wtAttr = cnsNode.listAttr()[-2:]
        
        condNode = pm.createNode('condition', name='cond_spine%02s_parent'%i)
        condNode.colorIfTrueR.set(1)
        condNode.colorIfFalseR.set(0)
        condNode.colorIfTrueG.set(0)
        condNode.colorIfFalseG.set(1)
        
        newAttr >> condNode.firstTerm
        condNode.outColorR >> wtAttr[0]
        condNode.outColorG >> wtAttr[1]


def replaceStartEndSpine():
    replacedSpine ={'01':['HIP_CTRL'],
                    '06':['SPINE_UPPER_CTRL']}
                   
    for jnt in replacedSpine:
        oldSpJnt = pm.PyNode('jSpine%s'%jnt)
        spJntPos = pm.xform(oldSpJnt, q=True, ws=True, translation=True)
        newSpJnt = pm.joint(name='temp_jSpine%s'%jnt, p=spJntPos)
        pm.parent( newSpJnt, 'bindSkelSpine')
        
        oldSpJnt.rename('old_%s'%oldSpJnt.name())
        newSpJnt.rename('jSpine%s'%jnt)
        pm.parentConstraint(replacedSpine[jnt][0], newSpJnt)


def wiggleJointChain(strPnt, endPnt, side='FL', chainPos='Upper'):
    '''
    create joint chain between two points (strPnt & endPnt). require name string of strPnt & endPnt
    '''
    strPos = pm.xform( strPnt, q=True, ws=True, translation=True )
    endPos = pm.xform( endPnt, q=True, ws=True, translation=True )
    
    if side.endswith('L'):
        sideLabel = 1
    elif side.endswith('R'):
        sideLabel = 2
        
    ikSpCrv = pm.curve( degree=2, editPoint=( strPos, endPos) )
    ikSpCrv.rename( 'wiggle_%s_%s_CRV'%(side, chainPos) )
    ikSpCrvShp = ikSpCrv.listRelatives(shapes=True)[0]
    pm.select(clear=True)
    
    jnt2pos = pm.pointOnCurve( ikSpCrv, pr=0.3333, turnOnPercentage=True)
    jnt3pos = pm.pointOnCurve( ikSpCrv, pr=0.6667, turnOnPercentage=True )
    
    jntPos = ( strPos, jnt2pos, jnt3pos, endPos )
    jntList = []
    for pnt in jntPos:
        jName = 'Wiggle_%s_%s_%02d'%(side, chainPos, jntPos.index(pnt)+1)
        newJoint = pm.joint(name=jName, p=pnt)
        newJoint.side.set(sideLabel)
        newJoint.__getattr__('type').set(18)
        newJoint.otherType.set(jName)
        jntList.append(newJoint)
        
    pm.joint( jntList[0], edit=True, orientJoint='xyz', secondaryAxisOrient='xup', children=True, zeroScaleOrient=True )
    
    ikHandle = pm.ikHandle( name='Wiggle_%s_%s_ikHandle'%(side, chainPos),
                            solver='ikSplineSolver', 
                            createCurve=False, 
                            curve=ikSpCrvShp, 
                            startJoint=jntList[0].name(), 
                            endEffector=jntList[-1].name(), 
                            rootOnCurve=False, 
                            createRootAxis=True, 
                            parentCurve=False )
    
    jntGrp = jntList[0].listRelatives(parent=True)[0]
    jntGrp.rename('Wiggle_%s_%s'%(side, chainPos))
    crvInfo = pm.createNode('curveInfo', name='crvInf_wiggle_%s_%s'%(side, chainPos))
    multDiv1 = pm.createNode('multiplyDivide', name='md_wiggle_%s_%s_01'%(side, chainPos))
    multDiv2 = pm.createNode('multiplyDivide', name='md_wiggle_%s_%s_02'%(side, chainPos))
    ikSpCrvShp.worldSpace >> crvInfo.inputCurve
    arcLgt = crvInfo.arcLength.get()
    multDiv1.input2X.set(arcLgt)
    multDiv1.operation.set(2)
    spacing = jntList[1].tx.get()
    multDiv2.input2X.set(spacing)
    multDiv1.outputX >> multDiv2.input1X
    crvInfo.arcLength >> multDiv1.input1X
    
    for jnt in jntList[1:]:
        multDiv2.outputX >> jnt.tx
    
    return ikSpCrvShp, ikSpCrv, ikHandle[0], jntGrp


def circleCtrl(ctrlName, radius=8, colorIdx=13):
    ctrl = pm.circle(name=ctrlName, radius=radius, normal=(1,0,0),constructionHistory=False)[0]
    ctrl.overrideEnabled.set(1)
    ctrl.overrideColor.set(colorIdx)
    ctrlGrp = pm.group(em=True, name='%s_GRP'%ctrlName)
    lockedAttrs = ('rx', 'ry', 'rz', 'sx', 'sy', 'sz', 'visibility')
    map(lambda attr:ctrl.__getattr__(attr).set(keyable=False, lock=True), lockedAttrs)
    pm.parent(ctrl, ctrlGrp)
    
    return ctrl, ctrlGrp


def addLimbWiggle():
    sides = ('FL', 'FR', 'BL', 'BR')
    
    wglMisc = []
    wglJnts = []
    wglCtrl = []
    
    for side in sides:
        if side.startswith('F'):
            limb = 'Arm'
            attachPoint = 'Clav'
            ctrlRadius = 4
            part = 'HAND'
            
        elif side.startswith('B'):
            limb = 'Leg'
            ctrlRadius = 8
            attachPoint = 'HipB'
            part = 'FOOT'
        
        if side.endswith('L'):
            ctrlColor = 13
            twistMult = 1
        elif side.endswith('R'):
            ctrlColor = 14
            twistMult = -1
            
        strJnt = pm.PyNode('j%s%s01'%(limb,side))
        midJnt = pm.PyNode('j%s%s02'%(limb,side))
        endJnt = pm.PyNode('j%s%s03'%(limb,side))
        
        upperWiggle = wiggleJointChain(strJnt, midJnt, side=side, chainPos='Upper')
        lowerWiggle = wiggleJointChain(midJnt, endJnt, side=side, chainPos='Lower')
        
        if limb == 'Arm':
            twistJnt = pm.PyNode('jArm%s02f'%side)
            twistMD = pm.createNode('multiplyDivide', name='md_armTwistMult_%s'%side)
            twistMD.input2X.set(twistMult)
            twistJnt.rx >> twistMD.input1X
            twistMD.outputX >> lowerWiggle[2].twist
        
        cls1 = pm.cluster(upperWiggle[1].cv[0], name='clsWiggle_%s_%s_01'%(limb, side))[1]
        cls2 = pm.cluster(upperWiggle[1].cv[1], name='clsWiggle_%s_%s_02'%(limb, side))[1]
        cls3 = pm.cluster((upperWiggle[1].cv[2],lowerWiggle[1].cv[0]), name='clsWiggle_%s_%s_03'%(limb, side))[1]
        cls4 = pm.cluster(lowerWiggle[1].cv[1], name='clsWiggle_%s_%s_04'%(limb, side))[1]
        cls5 = pm.cluster(lowerWiggle[1].cv[2], name='clsWiggle_%s_%s_05'%(limb, side))[1]
        
        ctrlList = []
        ctrlDict = {1:(cls2, strJnt),
                    2:(cls3, midJnt),
                    3:(cls4, midJnt)}
                    
        for idx in ctrlDict:
            ctrl = circleCtrl('Wiggle_%s_%s_%02d_CTRL'%(limb, side, idx), radius=ctrlRadius, colorIdx=ctrlColor)
            tempPrCns = pm.parentConstraint(ctrlDict[idx][0], ctrl[1], mo=False)
            pm.delete(tempPrCns)
            tempOrCns = pm.orientConstraint(ctrlDict[idx][1], ctrl[1], mo=False)
            pm.delete(tempOrCns)
            ctrlList.append(ctrl)
        
        pm.parentConstraint(strJnt, cls1)
        pm.parentConstraint(ctrlList[0][0], cls2)
        pm.parentConstraint(ctrlList[1][0], cls3)
        pm.parentConstraint(ctrlList[2][0], cls4)
        pm.parentConstraint(endJnt, cls5)
        
        pm.parentConstraint(midJnt, ctrlList[1][1])
        grpCns1 = pm.pointConstraint(strJnt, ctrlList[0][1], mo=True)
        pm.pointConstraint(ctrlList[1][0], ctrlList[0][1], mo=True)
        pm.orientConstraint(strJnt, ctrlList[0][1], mo=True)
        # grpCns1 = pm.parentConstraint('j%s%s02'%(attachPoint, side[1]), ctrlList[0][1], mo=True)
        # pm.parentConstraint(ctrlList[1][0], ctrlList[0][1])
        grpCns2 = pm.pointConstraint(ctrlList[1][0], ctrlList[2][1])
        pm.pointConstraint(endJnt, ctrlList[2][1])
        pm.orientConstraint(midJnt, ctrlList[2][1], mo=True)
        # map(lambda cns:cns.interpType.set(2), (grpCns1, grpCns2))
        pm.parentConstraint(strJnt, upperWiggle[-1])
        pm.parentConstraint(ctrlList[1][0], lowerWiggle[-1])
        
        uWglMiscGrp = pm.group(upperWiggle[1:3], name='UpperWiggle_%s_%s_GRP'%(limb, side))
        lWglMiscGrp = pm.group(lowerWiggle[1:3], name='LowerWiggle_%s_%s_GRP'%(limb, side))
        wglMisc.append(uWglMiscGrp)
        wglMisc.append(lWglMiscGrp)
        wglMisc.append((cls1,cls2,cls3,cls4,cls5))
        wglJnts.append(upperWiggle[-1])
        wglJnts.append(lowerWiggle[-1])
        map(lambda ctrl:wglCtrl.append(ctrl[1]), ctrlList)
        
        ctrlObj = pm.PyNode('%s_%s_FKIK_CTRL'%(part,side))
        pm.addAttr(ctrlObj, sn='wiggleControl', at='bool')
        ctrlObj.wiggleControl.set(0, keyable=True, lock=False)
        map(lambda ctrl:ctrlObj.wiggleControl >> ctrl[1].visibility, ctrlList)
        
    wglMiscGrp = pm.group(wglMisc, name='Wiggle_Misc_GRP', parent='misc')
    wglJntsGrp = pm.group(wglJnts, name='Wiggle_Jnts_GRP', parent='bindSkel')
    wglCtrlGrp = pm.group(wglCtrl, name='Wiggle_CTRL_GRP', parent='controls')
    pm.scaleConstraint('CONSTRAIN', wglCtrlGrp, mo=True)
    
    
def addLightingGroup(lightGrp=3):
    
    rootLgtGrp = pm.group(em=True, name='Lightings', parent='All')
    
    for i in range(1,lightGrp+1):
        lgtGrp = pm.group(em=True, name='Lights01', parent=rootLgtGrp)
        pm.addAttr(lgtGrp, sn='follow', at='enum', en='HEAD:BODY:BLOCK')
        lgtGrp.follow.set(2, keyable=True, lock=False)
        parCns = pm.parentConstraint(('jHead02','SPINE_UPPER_CTRL','BLOCK'), lgtGrp)
        pm.scaleConstraint('CONSTRAIN', lgtGrp)
        toLockAttr = ('tx','ty','tz','rx','ry','rz','sx','sy','sz')
        map(lambda attr:lgtGrp.__getattr__(attr).set(keyable=False, lock=True), toLockAttr)
        wgtList = parCns.listAttr()[-3:]
        
        for wgt in wgtList:
            index = wgtList.index(wgt)
            condNode = pm.createNode('condition', name='cond_lgtGrp%02d'%(index+1))
            condNode.secondTerm.set(index)
            condNode.colorIfTrueR.set(1)
            condNode.colorIfFalseR.set(0)
            lgtGrp.follow >> condNode.firstTerm
            condNode.outColorR >> wgt
        
        
        
