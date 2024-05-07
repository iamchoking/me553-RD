import xmltodict
import yaml
import os 

def find_joint(d,name):
    for j in d['robot']['joint']:
        if j['@name'] == name:
            return j
    return None


def find_link(d,name):
    for l in d['robot']['link']:
        if l['@name'] == name:
            return l
    return None

# generate Trans profile
# template (fixed) :   
# <NAME> (fixed) <origin rpy="{R} {P} {Y}" xyz="{X} {Y} {Z}"/>
# tempT.setProfile(x,y,z,R,P,Y);

# template (active): (usually, gcIndex = gvIndex+1)
# <<NAME>> ({rev/prsm}) <axis xyz="{X} {Y} {Z}"/> (gc[13])
# tempT.setProfile(x,y,z,R,P,Y,newTyp,ax,ay,az,gcIndex,gvIndex);

def generate_trans_profile(d,name,gcIdx = -1,pSpace = 2):
    j_raw = find_joint(d,name)

    typ  = j_raw['@type'][0]
    name = j_raw['@name']

    [x,y,z] = j_raw['origin']['@xyz'].split(" ")
    [R,P,Y] = j_raw['origin']['@rpy'].split(" ")
    s_origin = f'<origin rpy="{R} {P} {Y}" xyz="{x} {y} {z}"/>'

    if typ != 'f':
        [ax,ay,az] = j_raw['axis']['@xyz'].split(" ")
        s_axis = f'<axis xyz="{ax} {ay} {az}/>" (gc[{gcIdx}])'

    s_comment = ""
    s_profile = ""

    if typ == 'f':
        s_comment = f'{" "*pSpace}// <{name}> ({j_raw["@type"]}) {s_origin}'
        s_profile = f'{" "*pSpace}tempT.setProfile({x}, {y}, {z},   {R}, {P}, {Y});'

    else:
        s_comment = f'{" "*pSpace}// <<{name}>> ({j_raw["@type"]}) {s_origin} {s_axis}'
        s_profile = f'{" "*pSpace}tempT.setProfile({x}, {y}, {z},   {R}, {P}, {Y}, \'{typ}\',  {ax}, {ay}, {az}, {gcIdx}, {gcIdx-1});'
    

    return f'{s_comment}\n{s_profile}'


# generate Inertia profile
# template:
# // [LH_HAA]
# // <origin rpy="0 0 0" xyz="-0.063 7e-05 0.00046"/>
# // <mass value="2.04"/>
# // <inertia ixx="0.001053013" ixy="4.527e-05" ixz="8.855e-05" iyy="0.001805509" iyz="9.909e-05" izz="0.001765827"/>
# tempI.setProfile(x,y,z,R,P,Y,mass,ixx, ixy, ixz, iyy, iyz, izz)

def generate_inertia_profile(d,name,pSpace = 2):
    l_raw = find_link(d,name)
    l_inert = l_raw['inertial']
    l_r = l_inert['inertia']

    name = l_raw['@name']
    [x,y,z] = l_inert['origin']['@xyz'].split(" ")
    [R,P,Y] = l_inert['origin']['@rpy'].split(" ")
    mass = l_inert['mass']['@value']
    [ixx,ixy,ixz,iyy,iyz,izz] = [l_r['@ixx'],l_r['@ixy'],l_r['@ixz'],l_r['@iyy'],l_r['@iyz'],l_r['@izz']]

    s_label   = f'{" "*pSpace}// [{name}] <origin rpy="{R} {P} {Y}" xyz="{x} {y} {z}"/>'
    s_inertia = f'{" "*pSpace}// mass value="{mass}"/> <inertia ixx="{ixx}" ixy="{ixy}" ixz="{ixz}" iyy="{iyy}" iyz="{iyy}" izz="{izz}"/>'
    s_profile = f'{" "*pSpace}tempI.setProfile({x},{y},{z},  {R},{P},{Y},  {mass},  {ixx}, {ixy}, {ixz}, {iyy}, {iyz}, {izz});'

    return f'{s_label}\n{s_inertia}\n{s_profile}'

def main():
    dir_path = os.path.dirname(os.path.realpath(__file__))
    anymal_path = dir_path+"/../../resource/anymal_c/urdf/anymal.urdf"
    xml_string = open(anymal_path,"r").read()
    # print(xml_string)

    robot_dict=xmltodict.parse(xml_string)
    # print(find_link(robot_dict,"LH_HAA"))
    # print(find_joint(robot_dict,"LH_HAA"))

    # # print(robot_dict)
    # links = robot_dict['robot']['link']
    # joints = robot_dict['robot']['joint']

    # print("\n\n----LINKS----\n")

    # for link in links:
    #     print(link['@name'])
    #     try:
    #         print(link['inertial'])
    #     except:
    #         print("(no inertial property)")

    # print("\n\n----JOINTS----\n")

    # for j in robot_dict['robot']['joint']:
    #     print(j['@name'])
    #     print(j.keys())
    #     print(j['@type'] + " ")
    #     print(j['origin']['@xyz'].split(" "))
    #     try:
    #         print(j['axis']['@xyz'].split(" "))
    #     except:
    #         print("no actuated axis")

    # print(generate_trans_profile(robot_dict,"LH_shank_fixed_LH_FOOT",13,4))
    print(generate_inertia_profile(robot_dict,"LH_FOOT",4))

main()