import sys,os
sys.path.append(os.environ["MPP_PATH"]+"mpp-environment/mpp")

class boxUrdfFactory():
        def __init__(self):
                self.ctr=0

        def header(self):
                s =''
                s+='<?xml version="1.0"?>\n'
                s+='<robot name="wall_description">\n'
                s+='\n'
                s+='  <link name="base_link">\n'
                s+='    <visual>\n'
                s+='      <geometry>\n'
                s+='        <box size="0 0 0"/>\n'
                s+='      </geometry>\n'
                s+='    <origin rpy="0 0 0" xyz="0 0 0"/>\n'
                s+='    </visual>\n'
                s+='  </link>\n'
                s+='\n'
                return s

        def footer(self):
                s =''
                s+='\n'
                s+='</robot>\n'
                return s

        def box(self,x,y,z,sx,sy,sz):
                s=''
                s+=' <link name="b'+str(self.ctr)+'">\n'
                s+='   <visual>\n'
                s+='     <geometry>\n'
                s+='       <box size="%f %f %f"/>\n' %(sx,sy,sz)
                s+='     </geometry>\n'
                s+='     <origin rpy="0 0 0" xyz="%f %f %f"/>\n' %(x,y,z)
                s+='   <material name="wall_colored">\n'
                s+='     <color rgba="0.6 0.1 0.1 1.000000"/>\n'
                s+='   </material>\n'
                s+='   </visual>\n'
                s+='   <collision>\n'
                s+='     <geometry>\n'
                s+='       <box size="%f %f %f"/>\n' %(sx,sy,sz)
                s+='     </geometry>\n'
                s+='     <origin rpy="0 0 0" xyz="%f %f %f"/>\n' %(x,y,z)
                s+='   </collision>\n'
                s+=' </link>\n'
                s+='\n'
                s+=' <joint name="j'+str(self.ctr)+'" type="fixed">\n'
                s+='   <parent link="base_link"/>\n'
                s+='   <child link="b'+str(self.ctr)+'"/>\n'
                s+=' </joint>\n'
                s+='\n'
                self.ctr+=1
                return s

        def box2d(self,x,y,z,sx,sy):
                sz = 2*z
                s=''
                s+=' <link name="b'+str(self.ctr)+'">\n'
                s+='   <visual>\n'
                s+='     <geometry>\n'
                s+='       <box size="%f %f %f"/>\n' %(sx,sy,sz)
                s+='     </geometry>\n'
                s+='     <origin rpy="0 0 0" xyz="%f %f %f"/>\n' %(x,y,z)
                s+='   <material name="wall_colored">\n'
                s+='     <color rgba="0.6 0.1 0.1 1.000000"/>\n'
                s+='   </material>\n'
                s+='   </visual>\n'
                s+='   <collision>\n'
                s+='     <geometry>\n'
                s+='       <box size="%f %f %f"/>\n' %(sx,sy,sz)
                s+='     </geometry>\n'
                s+='     <origin rpy="0 0 0" xyz="%f %f %f"/>\n' %(x,y,z)
                s+='   </collision>\n'
                s+=' </link>\n'
                s+='\n'
                s+=' <joint name="j'+str(self.ctr)+'" type="fixed">\n'
                s+='   <parent link="base_link"/>\n'
                s+='   <child link="b'+str(self.ctr)+'"/>\n'
                s+=' </joint>\n'
                s+='\n'
                self.ctr+=1
                return s

        def getCtr(self):
                return self.ctr
