import json
import numpy as np
import vtk
volume=slicer.util.getNode('NAVBENCH_public_MR_head')
surface=slicer.util.getNode('NAVBENCH_MR_intensity_surface_NOT_validated_segmentation')
pd=surface.GetPolyData()
b=pd.GetBounds()
t=vtk.vtkMatrix4x4()
surface.GetParentTransformNode().GetMatrixTransformToWorld(t)
T=np.array([[t.GetElement(i,j) for j in range(4)] for i in range(4)])
state=slicer.ur12eSimulation.state
links=state['link_transforms']
tip_base=np.array(state['tcp_pose']['position_m'])*1000
flange_base=np.array(links['tool0']['position_m'])*1000
tip_image=(np.linalg.inv(T)@np.r_[tip_base,1])[:3]
flange_image=(np.linalg.inv(T)@np.r_[flange_base,1])[:3]
tree=vtk.vtkOBBTree()
tree.SetDataSet(pd)
tree.BuildLocator()
hits=vtk.vtkPoints()
ids=vtk.vtkIdList()
tree.IntersectWithLine(flange_image,tip_image,hits,ids)
r={'volume_dimensions_voxels':list(volume.GetImageData().GetDimensions()),'volume_spacing_mm':list(volume.GetSpacing()),'head_surface_RAS_extents_mm':[b[2*i+1]-b[2*i] for i in range(3)],'head_parent_scale_singular_values':np.linalg.svd(T[:3,:3],compute_uv=False).tolist(),'tool_length_mm':float(np.linalg.norm(tip_base-flange_base)),'tool_tip_image_RAS_mm':tip_image.tolist(),'needle_surface_intersections':hits.GetNumberOfPoints(),'distance_from_intersections_to_tip_mm':[float(np.linalg.norm(np.array(hits.GetPoint(i))-tip_image)) for i in range(hits.GetNumberOfPoints())],'feedback_source':state['source']}
for name,a,c in [('upper_arm','upper_arm_link','forearm_link'),('forearm','forearm_link','wrist_1_link')]:
    r[name+'_joint_origin_distance_mm']=float(np.linalg.norm(np.array(links[a]['position_m'])-links[c]['position_m']))*1000
(PROJECT/'output/demo_2026-10-31/head_geometry_measurements.json').write_text(json.dumps(r,indent=2),encoding='utf-8')
_result=r