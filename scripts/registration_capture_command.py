from pathlib import Path
p=slicer.registrationPractice
panel=slicer.util.mainWindow().findChild(qt.QDockWidget,'PanelDockWidget')
if panel:
    panel.setMinimumWidth(480)
    panel.setMaximumWidth(490)
p.dock.setMinimumWidth(250)
p.dock.setMaximumWidth(280)
slicer.util.mainWindow().showNormal()
slicer.util.mainWindow().resize(1680,960)
slicer.util.mainWindow().move(15,15)
slicer.app.processEvents()
slicer.util.resetThreeDViews()
assert slicer.util.saveScene(str(ROOT/'output/registration_2026-10-17/PointRegistration_before.mrb'))
slicer.app.processEvents()
assert slicer.util.mainWindow().grab().save(str(ROOT/'output/registration_2026-10-17/01_before_and_parameters.png'))
_result={'saved':'before scene and parameters screenshot'}
