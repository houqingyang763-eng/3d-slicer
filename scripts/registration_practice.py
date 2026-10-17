"""Reproducible paired-point registration exercise; run inside 3D Slicer."""
from pathlib import Path
import json
import numpy as np
import qt
import slicer
import vtk

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'output' / 'registration_2026-10-17'
OUT.mkdir(parents=True, exist_ok=True)


def matrix4(values):
    m = vtk.vtkMatrix4x4()
    for i in range(4):
        for j in range(4):
            m.SetElement(i, j, float(values[i, j]))
    return m


def points(name, xyz, color, prefix):
    node = slicer.mrmlScene.AddNewNodeByClass('vtkMRMLMarkupsFiducialNode', name)
    for i, p in enumerate(xyz):
        node.AddControlPoint(vtk.vtkVector3d(*p), f'{prefix}{i+1}')
    node.SetLocked(True)
    d = node.GetDisplayNode()
    d.SetSelectedColor(*color)
    d.SetGlyphScale(2.5)
    d.SetTextScale(1.6)
    return node


class RegistrationPractice:
    def __init__(self):
        self.dock = qt.QDockWidget('Point Registration Practice / SIMULATION', slicer.util.mainWindow())
        self.dock.setObjectName('RegistrationPracticeDock')
        body = qt.QWidget()
        layout = qt.QVBoxLayout(body)
        title = qt.QLabel('Paired-point rigid registration\nGreen = Fixed / Cyan = Moving\nCoordinates: RAS, millimetres')
        title.setWordWrap(True)
        layout.addWidget(title)
        self.info = qt.QLabel('')
        self.info.setWordWrap(True)
        for label, callback in [('1. Reset / prepare points', self.reset), ('2. Inspect paired landmarks', self.inspect), ('3. Compute rigid registration', self.compute), ('4. Apply MovingToFixed', self.apply), ('5. Independent validation', self.validate), ('6. Save scene + results', self.save)]:
            button = qt.QPushButton(label)
            button.connect('clicked()', callback)
            layout.addWidget(button)
        layout.addWidget(self.info)
        layout.addStretch(1)
        self.dock.setWidget(body)
        slicer.util.mainWindow().addDockWidget(qt.Qt.RightDockWidgetArea, self.dock)
        self.reset()

    def status(self, text):
        self.info.setText(text)
        slicer.app.layoutManager().threeDWidget(0).threeDView().cornerAnnotation().SetText(2, text)
        slicer.app.processEvents()

    def reset(self):
        slicer.mrmlScene.Clear(0)
        self.fixed_volume = slicer.util.loadVolume(str(ROOT / 'data' / 'MR-head.nrrd'), {'name':'Fixed_MRI'})
        self.moving_volume = slicer.modules.volumes.logic().CloneVolume(slicer.mrmlScene, self.fixed_volume, 'Moving_MRI_simulated')
        b = [0.0]*6
        self.fixed_volume.GetRASBounds(b)
        c = np.array([(b[i]+b[i+1])/2 for i in (0,2,4)])
        self.fixed_xyz = c + np.array([[-48,-35,-35],[45,-30,-32],[-38,42,-12],[35,38,18],[-15,-18,48],[25,12,55]],float)
        rz, ry = np.deg2rad(18), np.deg2rad(-9)
        Rz=np.array([[np.cos(rz),-np.sin(rz),0],[np.sin(rz),np.cos(rz),0],[0,0,1]])
        Ry=np.array([[np.cos(ry),0,np.sin(ry)],[0,1,0],[-np.sin(ry),0,np.cos(ry)]])
        self.truth_forward=np.eye(4)
        self.truth_forward[:3,:3]=Rz@Ry
        self.truth_forward[:3,3]=c+np.array([30,-22,15])-Rz@Ry@c
        self.truth_inverse=np.linalg.inv(self.truth_forward)
        self.moving_xyz=self.fixed_xyz@self.truth_forward[:3,:3].T+self.truth_forward[:3,3]
        ijk=vtk.vtkMatrix4x4()
        self.fixed_volume.GetIJKToRASMatrix(ijk)
        self.moving_volume.SetIJKToRASMatrix(matrix4(self.truth_forward @ slicer.util.arrayFromVTKMatrix(ijk)))
        self.fixed=points('Fixed_landmarks',self.fixed_xyz,(0.2,1,0.25),'F')
        self.moving=points('Moving_landmarks',self.moving_xyz,(0.1,0.85,1),'M')
        self.target_fixed=c+np.array([[0,0,0],[-12,22,15],[18,-12,28]])
        self.target_moving=self.target_fixed@self.truth_forward[:3,:3].T+self.truth_forward[:3,3]
        self.transform=slicer.mrmlScene.AddNewNodeByClass('vtkMRMLLinearTransformNode','MovingToFixed')
        self.cli=slicer.cli.createNode(slicer.modules.fiducialregistration, {'fixedLandmarks':self.fixed.GetID(),'movingLandmarks':self.moving.GetID(),'saveTransform':self.transform.GetID(),'transformType':'Rigid'})
        self.cli.SetName('PairedPoint_Rigid_CLI')
        self.before=np.linalg.norm(self.fixed_xyz-self.moving_xyz,axis=1)
        self.results={'date':'2026-10-17','version':slicer.app.applicationVersion,'coordinate_system':'RAS','units':'mm','number_of_pairs':6,'fixed_points':self.fixed_xyz.tolist(),'moving_points':self.moving_xyz.tolist(),'known_fixed_to_moving':self.truth_forward.tolist(),'expected_moving_to_fixed':self.truth_inverse.tolist(),'before_rms_mm':float(np.sqrt(np.mean(self.before**2))),'independent_fixed_targets':self.target_fixed.tolist()}
        lm=slicer.app.layoutManager()
        lm.setLayout(slicer.vtkMRMLLayoutNode.SlicerLayoutFourUpView)
        slicer.util.setSliceViewerLayers(background=self.fixed_volume,foreground=self.moving_volume,foregroundOpacity=0.5)
        slicer.util.resetSliceViews()
        slicer.util.setSliceViewerLayers(background=self.fixed_volume,foreground=self.moving_volume,foregroundOpacity=0.5)
        slicer.util.setSliceViewerLayers(label=None)
        slicer.util.resetThreeDViews()
        for name in lm.sliceViewNames():
            lm.sliceWidget(name).mrmlSliceNode().SetSliceVisible(True)
        slicer.util.setSliceViewerLayers(background=self.fixed_volume,foreground=self.moving_volume,foregroundOpacity=0.5)
        slicer.util.selectModule('FiducialRegistration')
        slicer.modules.fiducialregistration.widgetRepresentation().setEditedNode(self.cli)
        self.applied=False
        slicer.util.mainWindow().setWindowTitle('3D Slicer - Point Registration 2026-10-17')
        panel=slicer.util.mainWindow().findChild(qt.QDockWidget,'PanelDockWidget')
        if panel:
            panel.setMinimumWidth(480)
            panel.setMaximumWidth(490)
        self.dock.setMinimumWidth(250)
        self.dock.setMaximumWidth(280)
        slicer.util.mainWindow().showNormal()
        slicer.util.mainWindow().resize(1680,960)
        slicer.util.resetThreeDViews()
        self.status(f'BEFORE registration\n6 pairs / Rigid / mm\nRMS mismatch: {self.results["before_rms_mm"]:.3f} mm')

    def inspect(self):
        slicer.util.selectModule('Markups')
        slicer.modules.markups.widgetRepresentation().setEditedNode(self.moving)
        self.status('POINT PAIRS: F1 <-> M1 ... F6 <-> M6\nSame physical point, different frame.\nOrder must match. No collinear layout.')

    def compute(self):
        # CLI reads world points: detach first so repeated runs use original coordinates.
        self.moving.SetAndObserveTransformNodeID(None)
        self.moving_volume.SetAndObserveTransformNodeID(None)
        self.applied=False
        self.cli=slicer.cli.runSync(slicer.modules.fiducialregistration,self.cli, {'fixedLandmarks':self.fixed.GetID(),'movingLandmarks':self.moving.GetID(),'saveTransform':self.transform.GetID(),'transformType':'Rigid'})
        if self.cli.GetStatusString()!='Completed':
            raise RuntimeError(self.cli.GetErrorText() or self.cli.GetStatusString())
        m=vtk.vtkMatrix4x4()
        self.transform.GetMatrixTransformToParent(m)
        self.estimated=slicer.util.arrayFromVTKMatrix(m).copy()
        self.results['estimated_moving_to_fixed']=self.estimated.tolist()
        self.results['cli_status']='Completed'
        assert np.max(np.abs(self.estimated-self.truth_inverse))<1e-4, 'Unexpected transform direction or fit'
        slicer.util.selectModule('FiducialRegistration')
        slicer.modules.fiducialregistration.widgetRepresentation().setEditedNode(self.cli)
        self.status(f'COMPUTED: {self.cli.GetStatusString()}\nCLI RMS: {float(self.cli.GetParameterAsString("rms")):.8f} mm\nApply transform to MOVING nodes next.')

    def apply(self):
        if self.cli.GetStatusString()!='Completed':
            self.compute()
        else:
            m=vtk.vtkMatrix4x4()
            self.transform.GetMatrixTransformToParent(m)
            self.estimated=slicer.util.arrayFromVTKMatrix(m).copy()
            self.results['estimated_moving_to_fixed']=self.estimated.tolist()
        self.moving.SetAndObserveTransformNodeID(self.transform.GetID())
        self.moving_volume.SetAndObserveTransformNodeID(self.transform.GetID())
        self.applied=True
        slicer.util.selectModule('Data')
        slicer.util.resetSliceViews()
        self.status('AFTER applying MovingToFixed\nMoving MRI + Moving landmarks\nare children of the SAME transform.')

    def validate(self):
        if not self.applied:
            self.apply()
        registered=np.array([self.moving.GetNthControlPointPositionWorld(i) for i in range(6)])
        error=np.linalg.norm(registered-self.fixed_xyz,axis=1)
        target_registered=self.target_moving@self.estimated[:3,:3].T+self.estimated[:3,3]
        target_error=np.linalg.norm(target_registered-self.target_fixed,axis=1)
        self.results.update(after_pair_errors_mm=error.tolist(),after_fre_rms_mm=float(np.sqrt(np.mean(error**2))),independent_target_errors_mm=target_error.tolist(),independent_target_rms_mm=float(np.sqrt(np.mean(target_error**2))),cli_status_at_validation=self.cli.GetStatusString(),cli_rms_mm=float(self.cli.GetParameterAsString('rms')),transform_applied_to=[self.moving.GetName(),self.moving_volume.GetName()])
        self.results.setdefault('cli_status',self.cli.GetStatusString())
        assert max(error)<1e-4 and max(target_error)<1e-4
        slicer.util.selectModule('Transforms')
        slicer.modules.transforms.widgetRepresentation().setEditedNode(self.transform)
        self.status(f'VALIDATED / exact synthetic case\nFRE RMS: {self.results["after_fre_rms_mm"]:.8f} mm\n3 unused targets RMS: {self.results["independent_target_rms_mm"]:.8f} mm\nNot measured clinical accuracy.')
        return self.results

    def save(self):
        self.validate()
        for n in [self.fixed,self.moving,self.transform]:
            suffix='.h5' if n==self.transform else '.mrk.json'
            assert slicer.util.saveNode(n,str(OUT/(n.GetName()+suffix)))
        (OUT/'results.json').write_text(json.dumps(self.results,ensure_ascii=False,indent=2),encoding='utf-8')
        assert slicer.util.saveScene(str(OUT/'PointRegistration_after.mrb'))
        return self.results


def start():
    if hasattr(slicer,'classroomPractice'):
        slicer.util.mainWindow().removeDockWidget(slicer.classroomPractice.dock)
        slicer.classroomPractice.dock.deleteLater()
        del slicer.classroomPractice
    if hasattr(slicer,'registrationPractice'):
        slicer.util.mainWindow().removeDockWidget(slicer.registrationPractice.dock)
        slicer.registrationPractice.dock.deleteLater()
    slicer.registrationPractice=RegistrationPractice()
    return slicer.registrationPractice


if __name__=='__main__':
    start()
