"""Run inside the prepared classroom; save noise and wrong-pair controls."""
import csv
import json
import numpy as np
import slicer
import vtk
from registration_practice import OUT, points
p=slicer.registrationPractice
cases=[]
for name,xyz in [('noise_sigma_0.8mm',p.moving_xyz+np.random.default_rng(20261017).normal(0,0.8,p.moving_xyz.shape)),('cycle_pairs_1_2_3',p.moving_xyz[[1,2,0,3,4,5]])]:
    moving=points('Check_'+name,xyz,(1,0.6,0),'C')
    moving.GetDisplayNode().SetVisibility(False)
    transform=slicer.mrmlScene.AddNewNodeByClass('vtkMRMLLinearTransformNode','CheckTransform_'+name)
    cli=slicer.cli.runSync(slicer.modules.fiducialregistration,None,{'fixedLandmarks':p.fixed.GetID(),'movingLandmarks':moving.GetID(),'saveTransform':transform.GetID(),'transformType':'Rigid'})
    assert cli.GetStatusString()=='Completed',cli.GetErrorText()
    m=vtk.vtkMatrix4x4()
    transform.GetMatrixTransformToParent(m)
    T=slicer.util.arrayFromVTKMatrix(m)
    fre=np.linalg.norm(xyz@T[:3,:3].T+T[:3,3]-p.fixed_xyz,axis=1)
    target_error=np.linalg.norm(p.target_moving@T[:3,:3].T+T[:3,3]-p.target_fixed,axis=1)
    cases.append({'case':name,'fre_rms_mm':float(np.sqrt(np.mean(fre**2))),'heldout_target_rms_mm':float(np.sqrt(np.mean(target_error**2))),'cli_status':cli.GetStatusString(),'estimated_moving_to_fixed':T.tolist()})
    for node in [cli,transform,moving]:
        slicer.mrmlScene.RemoveNode(node)
p.results['controls']=cases
with (OUT/'point_pairs_RAS_mm.csv').open('w',encoding='utf-8',newline='') as f:
    writer=csv.writer(f)
    writer.writerow(['pair','fixed_R','fixed_A','fixed_S','moving_R','moving_A','moving_S'])
    for i,(fixed,moving) in enumerate(zip(p.fixed_xyz,p.moving_xyz)):
        writer.writerow([i+1,*fixed,*moving])

# A second, uncluttered view of points before/after. Both are native GUI captures.
lm=slicer.app.layoutManager()
lm.setLayout(slicer.vtkMRMLLayoutNode.SlicerLayoutOneUp3DView)
for name in lm.sliceViewNames():
    lm.sliceWidget(name).mrmlSliceNode().SetSliceVisible(False)
p.moving.SetAndObserveTransformNodeID(None)
p.inspect()
for button in slicer.util.findChildren(slicer.modules.markups.widgetRepresentation(),className='ctkCollapsibleButton'):
    if 'control points' in button.text.lower():
        button.collapsed=False
slicer.util.resetThreeDViews()
slicer.app.processEvents()
assert slicer.util.mainWindow().grab().save(str(OUT/'06_point_cloud_before.png'))
p.moving.SetAndObserveTransformNodeID(p.transform.GetID())
p.status('AFTER: F1=M1 ... F6=M6\nOnly moving nodes transformed.\nPerfect synthetic data, not clinical accuracy.')
slicer.util.resetThreeDViews()
slicer.app.processEvents()
assert slicer.util.mainWindow().grab().save(str(OUT/'07_point_cloud_after.png'))
lm.setLayout(slicer.vtkMRMLLayoutNode.SlicerLayoutFourUpView)
for name in lm.sliceViewNames():
    lm.sliceWidget(name).mrmlSliceNode().SetSliceVisible(True)
p.save()

# Independently reload the two saved scenes and inspect actual MRML hierarchy.
reload_checks=[]
for filename,expected_parent in [('PointRegistration_before.mrb',False),('PointRegistration_after.mrb',True)]:
    slicer.mrmlScene.Clear(0)
    assert slicer.util.loadScene(str(OUT/filename))
    slicer.app.processEvents()
    moving=slicer.util.getNode('Moving_landmarks')
    volume=slicer.util.getNode('Moving_MRI_simulated')
    assert bool(moving.GetTransformNodeID())==expected_parent
    assert bool(volume.GetTransformNodeID())==expected_parent
    errors=np.linalg.norm(np.array([moving.GetNthControlPointPositionWorld(i) for i in range(6)])-p.fixed_xyz,axis=1)
    rms=float(np.sqrt(np.mean(errors**2)))
    assert (rms<1e-4) if expected_parent else (rms>40)
    reload_checks.append({'scene':filename,'moving_nodes_under_transform':expected_parent,'pair_rms_mm':rms,'loaded':True})
p.fixed=slicer.util.getNode('Fixed_landmarks')
p.moving=slicer.util.getNode('Moving_landmarks')
p.fixed_volume=slicer.util.getNode('Fixed_MRI')
p.moving_volume=slicer.util.getNode('Moving_MRI_simulated')
p.transform=slicer.util.getNode('MovingToFixed')
p.cli=slicer.util.getNode('PairedPoint_Rigid_CLI')
p.results['reload_checks']=reload_checks
slicer.app.processEvents()
p.save()
_result={'controls':cases,'reload_checks':reload_checks}
