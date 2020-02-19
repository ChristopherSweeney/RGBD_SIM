
from director import transformUtils
from director import ioUtils
from director import filterUtils
from director import transformUtils
import numpy as np
from director import vtkAll as vtk
import Image
import yaml
import os

class CameraPoses(object):

    def __init__(self, posegraphFile=None):
        self.posegraphFile = posegraphFile

        if self.posegraphFile is not None:
            self.loadCameraPoses(posegraphFile)


    def loadCameraPoses(self, posegraphFile):
        data = np.loadtxt(posegraphFile)
        self.poseTimes = np.array(data[:,0]*1e6, dtype=int)
        self.poses = []
        for pose in data[:,1:]:
            pos = pose[:3]
            quat = pose[6], pose[3], pose[4], pose[5] # quat data from file is ordered as x, y, z, w
            self.poses.append((pos, quat))

    def getCameraPoseAtUTime(self, utime):
        idx = np.searchsorted(self.poseTimes, utime, side='left')
        if idx == len(self.poseTimes):
            idx = len(self.poseTimes) - 1

        (pos, quat) = self.poses[idx]
        return transformUtils.transformFromPose(pos, quat)

def setCameraTransform(camera, transform):
    '''Set camera transform so that view direction is +Z and view up is -Y'''
    origin = np.array(transform.GetPosition())
    axes = transformUtils.getAxesFromTransform(transform)
    camera.SetPosition(origin)
    camera.SetFocalPoint(origin+axes[2])
    camera.SetViewUp(-axes[1])

def setCameraInstrinsicsAsus(camera):
    principalX = 320.0
    principalY = 240.0
    focalLength = 528.0
    setCameraIntrinsics(camera, principalX, principalY, focalLength)

def setCameraIntrinsics(camera, principalX, principalY, focalLength):
    '''Note, call this function after setting the view dimensions'''
    imageWidth = 640
    imageHeight = 480
    wcx = -2*(principalX - float(imageWidth)/2) / imageWidth
    wcy =  2*(principalY - float(imageHeight)/2) / imageHeight
    viewAngle = focalLengthToViewAngle(focalLength, imageHeight)

    camera.SetWindowCenter(wcx, wcy)
    camera.SetViewAngle(viewAngle)

#this function is off by a little
def focalLengthToViewAngle(focalLength, imageHeight):
    '''Returns a view angle in degrees that can be set on a vtkCamera'''
    return np.degrees(2.0 * np.arctan2(imageHeight/2.0, focalLength))

#raycast normal aqusition is very slow
def encode_normal_rgb(renderer, height, width, pickType='cells', tolerance=0.05):
  #picktype is one of ('points', 'cells', 'render')
  image = np.zeros((height,width,3))
  for i in range(height):
    print i
    for j in range(width):
      # add some rgb conversion step, maybe png writer does that???
      picker = vtk.vtkPointPicker()
      picker.Pick(displayPoint[0], displayPoint[1], 0, renderer)
      #pickedProp = picker.GetViewProp()
      #pickedPoint = np.array(picker.GetPickPosition())
      #pickedDataset = pickedProp.GetMapper().GetInput() if isinstance(pickedProp, vtk.vtkActor) else None
      pointId = picker.GetPointId()
      normals = pickedDataset.GetPointData().GetNormals()
      if normals:
        print "no normals"
        pickedNormal = np.array(normals.GetTuple3(pointId))
        return None

def getFirstFrameToWorldTransform(transformsFile):
    if os.path.isfile(transformsFile):
        stream = file(transformsFile)
        transformYaml = yaml.load(stream)
        pose = transformYaml['firstFrameToWorld']
        transform = transformUtils.transformFromPose(pose[0], pose[1])
        return transform
    else:
        return vtk.vtkTransform()

def set_up_camera_params(camera):
  #setup camera calibration
  setCameraInstrinsicsAsus(camera)
  setCameraTransform(camera, vtk.vtkTransform())
  camera.SetWindowCenter(0,0)
  kClippingPlaneFar = 3.5
  kClippingPlaneNear = .2
  camera.SetClippingRange(kClippingPlaneNear, kClippingPlaneFar)
  camera.SetViewAngle(48.8879)
  kA = kClippingPlaneFar / (kClippingPlaneFar - kClippingPlaneNear)
  kB = -kA * kClippingPlaneNear

class Objects():
    def __init__(self, path=None, objects_file=None):
        self.path= path
        self.objects_file = objects_file
        self.objects = {} # dict of Actors
        self.poses = {}
        self.poses_perturb = {}
        self.icp_poses ={}
        self.objectToFirstFrame={}

    def loadObjectMeshes(self,registrationResultFilename,renderer,shader=None,keyword=None):
      self.registrationResultFilename = registrationResultFilename
      stream = file(self.path+registrationResultFilename)
      registrationResult = yaml.load(stream)
      self.firstFrameToWorldTransform = getFirstFrameToWorldTransform(self.path+'/transforms.yaml')
      for objName, data in registrationResult.iteritems():
          objectMeshFilename = self.objects_file+"/"+data['filename'] 
          if not keyword or keyword in objectMeshFilename:
            self.poses[objName] = data['pose']
            self.poses_perturb[objName] = data['pose']
            self.poses_perturb[objName][0][0] += .02
            self.poses_perturb[objName][0][1] += .02
            self.poses_perturb[objName][0][2] += .02
            self.poses_perturb[objName][1][0] += .02
            self.poses_perturb[objName][1][1] += .02
            self.poses_perturb[objName][1][2] += .02
            self.poses_perturb[objName][1][3] += .02

            self.objectToFirstFrame[objName] = transformUtils.transformFromPose(self.poses_perturb[objName][0], self.poses_perturb[objName][1])
            poly = ioUtils.readPolyData(objectMeshFilename)
            poly = filterUtils.transformPolyData(poly, self.objectToFirstFrame[objName])
            mapper = vtk.vtkPolyDataMapper()
            if shader: shader(mapper)
            mapper.SetInputData(poly)
            actor = vtk.vtkActor()
            actor.SetMapper(mapper)
            renderer.AddActor(actor)
            self.objects[objName] = actor

    def perturb_objects(self,rotx=1,roty=0,rotz=0,posex=0,posey=0,posez=0):
      for key in self.objects.keys():
          obj = self.objects[key] 
          pose = self.poses[key][0]
          orientation = self.poses[key][1]

          # obj.SetPosition(pose[0]+posex, pose[1]+posey, pose[2]+posez)
          # obj.RotateX(orientation[0]+rotx)
          # obj.RotateY(orientation[1]+roty)
          # obj.RotateZ(orientation[2]+rotz)

    def vtkICP(self, scene):
      for key in self.objects.keys():
        print key
        obj = self.objects[key]
        model = obj.GetMapper().GetInput()
        #set object prior loc here
        icp = vtk.vtkIterativeClosestPointTransform()
        icp.SetMaximumNumberOfIterations(5)
        #icp.StartByMatchingCentroidsOn()
        icp.SetSource(model)
        icp.SetTarget(scene)
        icp.GetLandmarkTransform().SetModeToRigidBody()
        icp.Modified()
        icp.Update()
        mat = icp.GetMatrix()
        t = vtk.vtkTransform()
        t.PostMultiply()
        t.SetMatrix(mat)
        modelToSceneTransform = t.GetLinearInverse()
        alignedModel = filterUtils.transformPolyData(model, modelToSceneTransform)
        new_pose = transformUtils.concatenateTransforms([self.objectToFirstFrame[key],modelToSceneTransform])
        pos,quat = transformUtils.poseFromTransform(new_pose)
        pos,quat = pos.tolist(),quat.tolist()
        self.icp_poses[key] = [pos,quat]

    def update_poses(self,renderer):
      stream = file(self.path+self.registrationResultFilename)
      registrationResult = yaml.load(stream)
      for objName, data in registrationResult.iteritems():
        if objName in self.icp_poses:# pose to update
          objectMeshFilename = self.objects_file+"/"+data['filename'] 
          objectToFirstFrame = transformUtils.transformFromPose(self.icp_poses[objName][0], self.icp_poses[objName][1])
          poly = ioUtils.readPolyData(objectMeshFilename)
          poly = filterUtils.transformPolyData(poly, objectToFirstFrame)
          mapper = vtk.vtkPolyDataMapper()
          mapper.SetInputData(poly)
          actor = self.objects[objName]
          renderer.RemoveActor(actor)
          actor.SetMapper(mapper)
          renderer.AddActor(actor)
    
    def reset(self):
        self.objects = {} # dict of Actors
        self.poses = {}
        self.poses_perturb = {}
        self.icp_poses ={}
        self.objectToFirstFrame={}


    def dump_icp_results(self,data_file='data.yml'):
        outfile = open(data_file, 'w')
        data={}
        data["ground_truth_pose"] = self.poses
        data["post_icp_pose"] = self.icp_poses 
        print self.poses
        print self.icp_poses
        yaml.dump(data, outfile, default_flow_style=False)
        outfile.close()

'''
def point_cloud_to_mesh():
    delaunay =vtk.vtkDelaunay2D()
  delaunay->SetAlpha(4.0);
>     delaunay->SetTolerance(0.0001);
>     delaunay->SetOffset(1.25);
>     delaunay->BoundingTriangulationOff();
>     delaunay->SetInputData(polyData);
>     delaunay->SetSourceData(polyData);
>     delaunay->Update();


def point_cloud_to_mesh(cloud):

poissonReconstruction.SetSamplesPerNode(1.0);
poissonReconstruction.SetDepth(10);          
poissonReconstruction.SetKernelDepth(6);      
poissonReconstruction.SetSolverDivide(10);    
poissonReconstruction.SetIsoDivide(10);  

vtkSmartPointer<vtkPoissonReconstruction> pR =
vtkSmartPointer<vtkPoissonReconstruction>::New();
    pR->SetInputConnection(polyDataNormals->GetOutputPort());
    pR->SetSamplesPerNode(1.0);
    pR->SetDepth(10);
    pR->SetKernelDepth(10);
    pR->SetSolverDivide(10);
    pR->SetIsoDivide(10);
    pR->Update(); 
    '''