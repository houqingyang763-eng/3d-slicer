import json, sys, time
import numpy as np
sys.path.insert(0,str(PROJECT/'slicer'))
import navigation_benchmark_scene as nav
import math
panel=slicer.ur12eSimulation
if panel.state.get('source')!='ros2_ursim' or not panel.state.get('feedback_valid'):
    raise RuntimeError('Current live URSim feedback required')
# Reopening the panel resets its editable fields. Display the controller's last
# target again without submitting a plan or movement command.
target=nav._pose_from_feedback(panel.state['target_pose'])
rot=target[:3,:3]
angles=np.rad2deg([math.atan2(rot[2,1],rot[2,2]),math.atan2(-rot[2,0],math.hypot(rot[0,0],rot[1,0])),math.atan2(rot[1,0],rot[0,0])])
panel.updating_target=True
try:
    for widget,value in zip(panel.position,target[:3,3]):
        widget.setDecimals(4)
        widget.setValue(float(value))
    for widget,value in zip(panel.angles,angles):
        widget.setDecimals(4)
        widget.setValue(float(value))
finally:
    panel.updating_target=False
panel.update_target()
head_nodes_before=[n.GetName() for n in slicer.util.getNodesByClass('vtkMRMLNode') if (n.GetName() or '').startswith(nav.PREFIX)]
r=json.loads((PROJECT/'output/navigation_benchmark/latest_result.json').read_text())
existing=slicer.util.getNodes(nav.PREFIX+'public_MR_head')
if not existing:
    surface_info=nav._volume_surface(r['head'],nav._matrix(r['head']['T_base_image_mm'],'display-only simulated image-to-base'))
else:
    surface_info={'reused_existing_head':True}
for n in slicer.util.getNodesByClass('vtkMRMLModelNode'):
    if (n.GetName() or '') == nav.PREFIX+'MR_intensity_surface_NOT_validated_segmentation':
        n.GetDisplayNode().SetVisibility(True)
        n.GetDisplayNode().SetOpacity(.4)
r['actual_tip_base_mm']=nav._pose_from_feedback(panel.state['tcp_pose']).tolist()
view=slicer.app.layoutManager().threeDWidget(0).threeDView()
view.cornerAnnotation().SetText(0,'PUBLIC MR HEAD | DISPLAY ONLY\nSimulated image-to-base; no physical registration\nNot validated skull segmentation')
out=PROJECT/'output/demo_2026-10-31'
nav.set_view(r,'platform')
nav._capture(out/'head_and_ursim_platform.png')
nav.set_view(r,'head')
nav._capture(out/'head_and_ursim_closeup.png')
_result={'head_nodes_before':head_nodes_before,'head_surface':surface_info,'robot_feedback_source':panel.state['source'],'robot_commands_sent':False,'head_transform_source':'existing navigation example simulated transform; display only','head_screenshot':str(out/'head_and_ursim_closeup.png')}
(out/'head_display_summary.json').write_text(json.dumps(_result,indent=2),encoding='utf-8')
