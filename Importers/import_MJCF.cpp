
// Check: ImportURDFSetup.cpp, OpenGLExampleBrowser.cpp

ImportMJCFSetup MJCF_Setup(options.m_guiHelper, options.m_option, options.m_fileName);

MJCF_Setup->initPhysics();
MJCF_Setup->resetCamera();
while (){
    MJCF_Setup.updateGraphics();
    MJCF_Setup.updateCamera(dg.upAxis);
    MJCF_Setup.stepSimulation(gFixedTimeStep);
    MJCF_Setup.renderScene();

}
