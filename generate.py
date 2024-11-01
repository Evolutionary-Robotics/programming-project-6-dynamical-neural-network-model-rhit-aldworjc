import pyrosim.pyrosim as ps

# Global parameters
l = 1 # length
w = 1 # width 
h = 1 # height 

x = 0
y = 0
z = 0.5

def Create_World():
    ps.Start_SDF("box.sdf")
    for i in range(10):
        ps.Send_Cube(name="Box",pos=[x,y,z],size=[l,w,h])
        z += h
        l = 0.9 * l
        w = 0.9 * w
        h = 0.9 * h
    ps.End()

def Create_Worm():
    ps.Start_URDF("body.urdf")
    ps.Send_Cube(name="Foot",pos=[x,y,z],size=[l,w,h]) # Parent
    ps.Send_Joint(name="Foot_Torso", parent="Foot", child="Torso", type="revolute", position = [0.5,0,1.0])
    ps.Send_Cube(name="Torso",pos=[0.5,0.0,0.5],size=[l,w,h]) # Child
    ps.Send_Joint(name="Torso_Tail", parent="Torso", child="Tail", type="revolute", position = [0,0,1.0])
    ps.Send_Cube(name="Tail",pos=[0.5,0.0,0.5],size=[l,w,h]) # Child2
    ps.End()

def Create_Stumble():
    ps.Start_URDF("stumble.urdf")
    ps.Send_Cube(name="Left_Foot",pos=[-0.25,0,z/2],size=[l/2,1,h/2]) # Parent
    ps.Send_Joint(name="Left_Foot_Torso", parent="Left_Foot", child="Torso", type="revolute", position = [0,0.5,0])
    ps.Send_Cube(name="Torso",pos=[1,0.5,0.25],size=[3,1,0.5]) # Child
    ps.Send_Joint(name="Right_Foot_Torso", parent="Torso", child="Right_Foot", type="revolute", position = [0,1.0,0])
    ps.Send_Cube(name="Right_Foot",pos=[-0.25,0.5,0.25],size=[0.5,1,h/2]) # Child
    ps.End()
    
def Create_Walker():
    ps.Start_URDF("walker.urdf")
    ps.Send_Cube(name="Torso",pos=[0,0,1.25],size=[2,1.5,0.5])
    ps.Send_Joint(name="LF_Motor", parent="Torso", child="LF_Foot", type="revolute", position = [0.75,0.5,1])
    ps.Send_Cube(name="LF_Foot",pos=[0,0,-0.5],size=[0.5,0.5,1])
    ps.Send_Joint(name="RF_Motor", parent="Torso", child="RF_Foot", type="revolute", position = [0.75,-0.5,1])
    ps.Send_Cube(name="RF_Foot",pos=[0,0,-0.5],size=[0.5,0.5,1])
    ps.Send_Joint(name="RB_Motor", parent="Torso", child="RB_Foot", type="revolute", position = [-0.75,-0.5,1])
    ps.Send_Cube(name="RB_Foot",pos=[0,0,-0.5],size=[0.5,0.5,1])
    ps.Send_Joint(name="LB_Motor", parent="Torso", child="LB_Foot", type="revolute", position = [-0.75,0.5,1])
    ps.Send_Cube(name="LB_Foot",pos=[0,0,-0.5],size=[0.5,0.5,1])
    ps.End()

Create_Walker()
