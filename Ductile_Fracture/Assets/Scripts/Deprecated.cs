

//public class Cuboid
//{ 
//    public Vector3[] point_positions = new Vector3[8];
//    public Node[] nodes = new Node[8];

//    public Cuboid(Vector3[] positions)
//    {
//        for(int i = 0; i < positions.Length; i++)
//        {
//            point_positions[i] = positions[i];
//        }
//    }

//    public List<Node> MakeNodesFromPoints(Vector3 world_position)
//    {
//        List<Node> nodes = new List<Node>();

//        foreach(Vector3 p in point_positions)
//        {
//            // TODO rather use Instantiate()
//            nodes.Add(new Node(p, p + world_position));
//        }

//        return nodes;
//    }

//    public List<Cuboid> Subdivide()
//    {
//        List<Cuboid> newCuboids = new List<Cuboid>();

//        Vector3[] connections = FindConnectedPoints(0);

//        Vector3 ConX = (connections[0] - point_positions[0]) / 2.0f;
//        Vector3 ConY = (connections[1] - point_positions[0]) / 2.0f;
//        Vector3 ConZ = (connections[2] - point_positions[0]) / 2.0f;

//        Cuboid referenceCuboid = CreateCuboid(point_positions[0], ConX, ConY, ConZ);
//        newCuboids.Add(referenceCuboid);

//        for(int i = 1; i < 8; i++)
//        {
//            newCuboids.Add(CreateCuboid(referenceCuboid.point_positions[i], ConX, ConY, ConZ));
//        }

//        return newCuboids;
//    }

//    public Vector3[] FindConnectedPoints(int index)
//    {
//        Vector3[] connections = new Vector3[3];

//        Vector3 start = point_positions[index];

//        for(int i = 0; i < 8; i++)
//        {
//            Vector3 current = point_positions[i];
//            if (i != index)
//            {
//                if (start.x == current.x && start.y == current.y)
//                {
//                    connections[2] = current;
//                    Debug.Assert(i == 1);
//                }
//                else if (start.x == current.x && start.z == current.z)
//                {
//                    connections[1] = current;
//                    Debug.Assert(i == 4);
//                }
//                else if (start.y == current.y && start.z == current.z)
//                {
//                    connections[0] = current;
//                    Debug.Assert(i == 3);
//                }
//            }
//        }

//        return connections;
//    }

//    public Cuboid CreateCuboid(Vector3 location, Vector3 connectedToLocationOnX, Vector3 connectedToLocationOnY, Vector3 connectedToLocationOnZ)
//    {
//        Vector3[] vertices = new Vector3[8];

//        vertices[0] = location;
//        vertices[1] = location + connectedToLocationOnZ;
//        vertices[2] = location + connectedToLocationOnX + connectedToLocationOnZ;
//        vertices[3] = location + connectedToLocationOnX;
//        vertices[4] = location + connectedToLocationOnY;
//        vertices[5] = location + connectedToLocationOnY + connectedToLocationOnZ;
//        vertices[6] = location + connectedToLocationOnX + connectedToLocationOnY + connectedToLocationOnZ;
//        vertices[7] = location + connectedToLocationOnX + connectedToLocationOnY;

//        return new Cuboid(vertices);
//    }

//    public List<Tetrahedron> GenerateTetrahedra()
//    {
//        List<Tetrahedron> tetrahedra = new List<Tetrahedron>();

//        tetrahedra.Add(new Tetrahedron(nodes[2], nodes[5], nodes[6], nodes[7]));
//        tetrahedra.Add(new Tetrahedron(nodes[0], nodes[4], nodes[5], nodes[7]));
//        tetrahedra.Add(new Tetrahedron(nodes[0], nodes[2], nodes[3], nodes[7]));
//        tetrahedra.Add(new Tetrahedron(nodes[0], nodes[1], nodes[2], nodes[5]));
//        tetrahedra.Add(new Tetrahedron(nodes[0], nodes[2], nodes[5], nodes[7]));

//        return tetrahedra;
//    }
//}




// properties of FractureSimulator-class

//[SerializeField]
//Transform sphere;

//[SerializeField]
//int Subdivisions;

//Mesh mesh;
//Vector3[] vertices = new Vector3[8];

//List<Cuboid> allcuboids = new List<Cuboid>();






// Start-Method()

// TODO add all attached tetrahedra to each node

//Time.fixedDeltaTime = 0.1f;

// retrieve local vertex positions
//MeshFilter meshFilter;
//if(gameObject.TryGetComponent(out meshFilter))
//{
//    mesh = meshFilter.mesh;
//    Vector3[] temp = mesh.vertices.Distinct().ToArray();
//    vertices[0] = temp[5];
//    vertices[1] = temp[3];
//    vertices[2] = temp[2];
//    vertices[3] = temp[4];
//    vertices[4] = temp[7];
//    vertices[5] = temp[1];
//    vertices[6] = temp[0];
//    vertices[7] = temp[6];

//    for (int i = 0; i < vertices.Length; i++)
//    {
//        vertices[i].x *= transform.localScale.x;
//        vertices[i].y *= transform.localScale.y;
//        vertices[i].z *= -transform.localScale.z; // IMPORTANT: Unity has the z-axis swapped, so we swap it back.
//    }

//    Cuboid main_cuboid = new Cuboid(vertices);
//    allcuboids.Add(main_cuboid);

//    // subdivide the cuboid into more cuboids
//    for (int subdiv_index = 0; subdiv_index < Subdivisions; subdiv_index++)
//    {
//        List<Cuboid> working_copy = new List<Cuboid>();
//        foreach (Cuboid c in allcuboids)
//        {
//            working_copy.Add(c);
//        }
//        allcuboids.Clear();
//        for (int cuboid_index = 0; cuboid_index < working_copy.Count; cuboid_index++)
//        {
//            Cuboid current_cuboid = working_copy[cuboid_index];
//            allcuboids.AddRange(current_cuboid.Subdivide());
//        }
//    }

//    // create nodes
//    for (int cuboid = 0; cuboid < allcuboids.Count; cuboid++)
//    {
//        allnodes.AddRange(allcuboids[cuboid].MakeNodesFromPoints(gameObject.transform.position));
//    }

//    // clear duplicates
//    for (int node_1 = 0; node_1 < allnodes.Count; node_1++)
//    {
//        int rerun_counter = 0;
//        for (int node_2 = 0; node_2 < allnodes.Count; node_2++)
//        {
//            if (node_1 == node_2) continue;

//            if (allnodes[node_1].world_pos == allnodes[node_2].world_pos)
//            {
//                allnodes.Remove(allnodes[node_2]);
//                node_2--;
//                rerun_counter++;
//            }
//        }
//        node_1 -= rerun_counter;
//    }

//    // find nodes for cuboids
//    for (int cuboid = 0; cuboid < allcuboids.Count; cuboid++)
//    {
//        for (int corner_pos = 0; corner_pos < 8; corner_pos++)
//        {
//            for (int node = 0; node < allnodes.Count; node++)
//            {
//                if (allcuboids[cuboid].point_positions[corner_pos] == allnodes[node].mat_pos)
//                {
//                    allcuboids[cuboid].nodes[corner_pos] = allnodes[node];
//                }
//            }
//        }
//    }

//    // create tetrahedra
//    for (int cuboid = 0; cuboid < allcuboids.Count; cuboid++)
//    {
//        alltetrahedra.AddRange(allcuboids[cuboid].GenerateTetrahedra());
//    }

//    // for each node, find all attached tetrahedra
//    foreach (Tetrahedron t in alltetrahedra)
//    {
//        foreach (Node n in t.GetNodes())
//        {
//            if(!n.attached_elements.Contains(t))
//                n.attached_elements.Add(t);
//        }
//    }

//    // find neighbors
//    Node[] nodes_t1;
//    Node[] nodes_t2;
//    for (int t1 = 0; t1 < alltetrahedra.Count; t1++)
//    {
//        for (int t2 = 0; t2 < alltetrahedra.Count; t2++)
//        {
//            if (t1 == t2) continue;
//            nodes_t1 = alltetrahedra[t1].GetNodes();
//            nodes_t2 = alltetrahedra[t2].GetNodes();
//            foreach (Node n1 in nodes_t1)
//            {
//                foreach (Node n2 in nodes_t2)
//                {
//                    if (n1.Equals(n2))
//                    {
//                        alltetrahedra[t1].AddNeighbor(alltetrahedra[t2], n1);
//                        alltetrahedra[t2].AddNeighbor(alltetrahedra[t1], n1);
//                    }
//                }
//            }
//        }
//    }
//}
//else
//{
//    // TESTCASE 1
//    Vector3 n1_mpos = new Vector3(-1.0f, 0.3f, 0.5f);
//    Node n1 = new Node(n1_mpos, gameObject.transform.position + n1_mpos);
//    allnodes.Add(n1);

//    Vector3 n2_mpos = new Vector3(-1.0f, 0.3f, -0.5f);
//    Node n2 = new Node(n2_mpos, gameObject.transform.position + n2_mpos);
//    allnodes.Add(n2);

//    Vector3 n3_mpos = new Vector3(1.0f, 0.3f, 0.0f);
//    Node n3 = new Node(n3_mpos, gameObject.transform.position + n3_mpos);
//    allnodes.Add(n3);

//    Vector3 n4_mpos = new Vector3(1.0f, -0.3f, 0.0f);
//    Node n4 = new Node(n4_mpos, gameObject.transform.position + n4_mpos);
//    allnodes.Add(n4);

//    Vector3 n5_mpos = new Vector3(1.0f, 0.6f, 0.0f);
//    Node n5 = new Node(n5_mpos, gameObject.transform.position + n5_mpos);
//    allnodes.Add(n5);

//    Tetrahedron t1 = new Tetrahedron(n1, n2, n3, n4);
//    alltetrahedra.Add(t1);

//    Tetrahedron t2 = new Tetrahedron(n1, n2, n3, n5);
//    alltetrahedra.Add(t2);

//    foreach (Tetrahedron t in alltetrahedra)
//    {
//        foreach (Node n in t.GetNodes())
//        {
//            if (!n.attached_elements.Contains(t))
//                n.attached_elements.Add(t);
//        }
//    }

//    foreach(Tetrahedron t1_i in alltetrahedra)
//    {
//        foreach(Tetrahedron t2_i in alltetrahedra)
//        {
//            if (t1_i.Equals(t2_i)) continue;
//            foreach(Node t1_i_node in t1_i.GetNodes())
//            {
//                foreach (Node t2_i_node in t2_i.GetNodes())
//                {
//                    if (t1_i_node.Equals(t2_i_node))
//                    {
//                        List<Node> list1 = new List<Node>();
//                        List<Node> list2 = new List<Node>();
//                        list1.Add(t1_i_node);
//                        list2.Add(t1_i_node);
//                        if(!t1_i.neighbors.ContainsKey(t2_i)) t1_i.neighbors.Add(t2_i, list1);
//                        if (!t2_i.neighbors.ContainsKey(t1_i)) t2_i.neighbors.Add(t1_i, list2);
//                    }
//                }
//            }
//        }
//    }

//    Debug.Log("There are " + allnodes.Count + " Nodes and " + alltetrahedra.Count + " Tetrahedra.");
//}
