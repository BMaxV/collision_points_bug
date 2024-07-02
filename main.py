from panda3d.core import CollisionTraverser, CollisionNode
from panda3d.core import CollisionHandlerQueue  # , CollisionRay
from panda3d.core import CollisionSphere  # CollisionHandlerPusher
from panda3d.core import CollideMask
from panda3d.core import NodePath
from panda3d.core import LVector3f, LPoint3f


def main():
    rd1 = setup(3)
    comp_d1 = {'0': {'sp': LPoint3f(1.6, 0, 0), 'cp': LPoint3f(3, 0, 0), 'ip': LPoint3f(1.5, 0, 0), 'cn': LVector3f(1, 0, 0)}, '1': {
        'sp': LPoint3f(1.5, 0, 0), 'cp': LPoint3f(0.1, 0, 0), 'ip': LPoint3f(1.6, 0, 0), 'cn': LVector3f(-1, 0, 0)}}
    print("these dicts are 'not equal'")
    print(rd1 == comp_d1)
    
    print(rd1)
    print(comp_d1)
    print("")
    for key1 in rd1:
        for key2 in rd1[key1]:
            v1=rd1[key1][key2]
            v2=comp_d1[key1][key2]
            if v1!=v2:
                print("")
                print("these are 'not equal'")
                print(key1,key2)
                print(v1,v2)
    

    print("")
    rd2 = setup(2.5)
    comp_d2 = {'0': {'sp': LPoint3f(1.6, 0, 0), 'cp': LPoint3f(2.5, 0, 0), 'ip': LPoint3f(1, 0, 0), 'cn': LVector3f(1, 0, 0)}, '1': {
        'sp': LPoint3f(1, 0, 0), 'cp': LPoint3f(0.1, 0, 0), 'ip': LPoint3f(1.6, 0, 0), 'cn': LVector3f(-1, 0, 0)}}

    print(rd2 == comp_d2)


def setup(x):
    node_root = NodePath("node_root")

    cTrav = CollisionTraverser()

    CH_queue = CollisionHandlerQueue()

    Col1 = CollisionNode('col1')
    Col1.addSolid(CollisionSphere(center=(0.1, 0, 0), radius=1.5))
    Col1.setFromCollideMask(CollideMask.bit(1))
    Col1.setIntoCollideMask(CollideMask.bit(1))
    Col1NP = NodePath(Col1)
    Col1NP.reparentTo(node_root)

    Col2 = CollisionNode('col2')
    Col2.addSolid(CollisionSphere(center=(x, 0, 0), radius=1.5))
    Col2.setFromCollideMask(CollideMask.bit(1))
    Col2.setIntoCollideMask(CollideMask.bit(1))
    Col2NP = NodePath(Col2)
    Col2NP.reparentTo(node_root)

    cTrav.addCollider(Col1NP, CH_queue)
    cTrav.addCollider(Col2NP, CH_queue)

    cTrav.traverse(node_root)

    entries = CH_queue.getEntries()
    CH_queue.clearEntries()

    m = len(entries)
    c = 0

    rd = {}
    while c < m:
        entry = entries[c]

        collisionargs = [entry]

        fromn = collisionargs[0].from_node
        inton = collisionargs[0].into_node

        surface_point = collisionargs[0].getSurfacePoint(node_root)
        contact_pos = collisionargs[0].getContactPos(node_root)
        interior_point = collisionargs[0].getInteriorPoint(node_root)
        collision_normal = collisionargs[0].getSurfaceNormal(node_root)

        result_l = [surface_point, contact_pos,
                    interior_point, collision_normal]
        my_d = {str(c): {"sp": surface_point, "cp": contact_pos,
                         "ip": interior_point, "cn": collision_normal}}
        rd.update(my_d)

        c += 1

    return rd


if __name__ == "__main__":
    main()
