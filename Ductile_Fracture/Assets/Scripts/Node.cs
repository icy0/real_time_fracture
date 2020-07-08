using System;
using UnityEngine;
using System.Collections.Generic;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Factorization;

public class Node : MonoBehaviour
{
    [SerializeField]
    private Transform node_prefab;

    [SerializeField]
    public bool crack_at_start;

    public Vector3 old_world_position;
    public Vector3 velocity;

    public Vector<float> fracture_plane_normal;
    public List<Tetrahedron> attached_elements = new List<Tetrahedron>();

    public List<Vector<float>> tensile_forces = new List<Vector<float>>();
    public List<Vector<float>> compressive_forces = new List<Vector<float>>();

    public void Start()
    {
        velocity = new Vector3(0.0f, 0.0f, 0.0f);
        old_world_position = transform.position;
    }

    public Tuple<List<Tetrahedron>, List<Tetrahedron>, List<Node>> Crack(Vector3 world_position_of_cube)
    {
        Vector<float> world_pos_of_node = Vector<float>.Build.DenseOfArray(new float[] { transform.position.x, transform.position.y, transform.position.z });
        Vector<float> world_pos_of_cube = Vector<float>.Build.DenseOfArray(new float[] { world_position_of_cube.x, world_position_of_cube.y, world_position_of_cube.z });
        TetrahedronBuilder tet_builder = GameObject.Find("FEM_Mesh").GetComponent<TetrahedronBuilder>();
        RelationManager relation_manager = GameObject.Find("FEM_Mesh").GetComponent<RelationManager>();

        // for each intersected tetrahedron, this dictionary holds a list of Tuples, where one Tuple describes the edge (item1 - item2) and the world_pos (item3) of the intersection
        Dictionary<Tetrahedron, List<Tuple<Node, Node, Vector<float>>>> two_point_intersected_tets = new Dictionary<Tetrahedron, List<Tuple<Node, Node, Vector<float>>>>();

        // for each three-way-intersected tetrahedron, this dictionary holds a list of tuples, where one Tuple describes the edge (item1 - item2) and the world_pos (item3) of the intersection and it holds
        // a bool which indicates, whether the fracture plane of the intersection is coplanar with a face of the tetrahedron or not.
        Dictionary<Tetrahedron, Tuple<bool, List<Tuple<Node, Node, Vector<float>>>>> three_point_intersected_tets = new Dictionary<Tetrahedron, Tuple<bool, List<Tuple<Node, Node, Vector<float>>>>>();

        // and there is a list for each non-intersected tetrahedron.
        List<Tetrahedron> not_intersected_tets = new List<Tetrahedron>();

        // we also collect old tetrahedra, which are going to be removed
        List<Tetrahedron> old_tetrahedra = new List<Tetrahedron>();

        // we collect new tetrahedra, which require a relations update
        List<Tetrahedron> new_tetrahedra = new List<Tetrahedron>();

        // and we collect newly created nodes, which also require a relations update
        List<Node> new_nodes = new List<Node>();

        foreach (Tetrahedron t in attached_elements)
        {
            // in here, we want to test each tetrahedron attached to this node for intersection with the
            // fracture plane. This can result in different cases, which we all need to treat differently.

            //      - no intersection (trivial)
            //      - fracture plane is coplanar to a face of the tetrahedron (trivial)
            //      - one edge of the tetrahedron lies inside the fracture plane (complex)
            //      - the fracture plane none of the above and cuts right through the tetrahedron (complex)

            // we can distinguish these cases by testing which edge of the tetrahedron intersects with the fracture plane.
            Tuple<bool /*is coplanar to face*/, List<Tuple<Node, Node, Vector<float>>>> intersection_data = t.Intersect(this, fracture_plane_normal);
            List<Tuple<Node, Node, Vector<float>>> intersection_points = intersection_data.Item2;

            switch (intersection_points.Count)
            {
                case 3: three_point_intersected_tets.Add(t, new Tuple<bool, List<Tuple<Node, Node, Vector<float>>>>(intersection_data.Item1 /*is coplanar to face*/, intersection_points)); break;
                case 2: two_point_intersected_tets.Add(t, intersection_points); break;
                case 0: not_intersected_tets.Add(t); break;
            }
            Debug.Assert(intersection_points.Count != 1, "Wrong Intersection data.");
        }

        GameObject kZeroPlus_go = null;
        GameObject kZeroMinus_go = null;

        Node kZeroPlus = null;
        Node kZeroMinus = null;
        
        // it is possible for a crack to not hit any tetrahedron, so we have to account for that.
        if(not_intersected_tets.Count > 0 || two_point_intersected_tets.Count > 0 || three_point_intersected_tets.Count > 0)
        {
            kZeroPlus_go = Instantiate(node_prefab.gameObject, transform.parent);
            kZeroPlus_go.transform.position = transform.position;
            kZeroPlus_go.transform.localPosition = transform.localPosition;

            kZeroMinus_go = Instantiate(node_prefab.gameObject, transform.parent);
            kZeroMinus_go.transform.position = transform.position;
            kZeroMinus_go.transform.localPosition = transform.localPosition;

            kZeroPlus = kZeroPlus_go.GetComponent<Node>();
            kZeroMinus = kZeroMinus_go.GetComponent<Node>();
        }

        // some debug information
        // ======================================================================================================================================
        Debug.Log("Crack-Info: There are " + two_point_intersected_tets.Count +
            " cases of two-point-intersections, " + three_point_intersected_tets.Count +
            " cases of three-point-intersections and " + not_intersected_tets.Count + " cases of non-intersected tetrahedra.");

        if (three_point_intersected_tets.Count > 0)
        {
            int count = 0;
            foreach (KeyValuePair<Tetrahedron, Tuple<bool, List<Tuple<Node, Node, Vector<float>>>>> kvp in three_point_intersected_tets)
            {
                if (kvp.Value.Item1) count++;
            }
            Debug.Log("Out of the " + three_point_intersected_tets.Count +
                " three-point-intersections, there are " + count + " which are coplanar to a face of the tetrahedron.");
        }
        // ======================================================================================================================================

        // for each non-hit tetrahedron:
        foreach (Tetrahedron not_hit_t in not_intersected_tets)
        {
            int index_of_n = MathUtility.GetIndexOf(not_hit_t, this);
            Debug.Assert(index_of_n != -1);

            // we can choose any node of the tetrahedron except n, because it is not cut by the fracture plane 
            // and so all nodes are inside the same halfspace of the fracture plane. Therefore (index_of_n + 1) % 4.
            if (MathUtility.IsOnPositiveSide(not_hit_t.nodes[(index_of_n + 1) % 4], world_pos_of_node, fracture_plane_normal))
            {
                not_hit_t.SwapNodes(this, kZeroPlus);
            }
            else
            {
                not_hit_t.SwapNodes(this, kZeroMinus);
            }
        }

        // for each three point intersected tetrahedron:
        foreach (KeyValuePair<Tetrahedron, Tuple<bool /*is coplanar to face*/, List<Tuple<Node, Node, Vector<float>>>>> intersected_tet in three_point_intersected_tets)
        {
            if (intersected_tet.Value.Item1 /*is coplanar to face*/)
            {
                // first collect the nodes which are on the fracture plane
                List<Node> face_nodes = new List<Node>();
                face_nodes.Add(this);

                // for each edge that was intersected by the fracture plane
                for (int i = 0; i < 3; i++)
                {
                    Tuple<Node, Node, Vector<float>> intersected_edge = intersected_tet.Value.Item2[i];
                    Vector<float> inters_edge_node_1 = Vector<float>.Build.DenseOfArray(new float[] { intersected_edge.Item1.transform.position.x, intersected_edge.Item1.transform.position.y, intersected_edge.Item1.transform.position.z });
                    Vector<float> inters_edge_node_2 = Vector<float>.Build.DenseOfArray(new float[] { intersected_edge.Item2.transform.position.x, intersected_edge.Item2.transform.position.y, intersected_edge.Item2.transform.position.z });
                    float distance_1 = (float)(inters_edge_node_1 - intersected_edge.Item3).L2Norm(); // distance between intersection point and inters_edge_node_1
                    float distance_2 = (float)(inters_edge_node_2 - intersected_edge.Item3).L2Norm(); // distance between intersection point and inters_edge_node_2

                    Node on_face = distance_1 < distance_2 ? intersected_edge.Item1 : intersected_edge.Item2; // whichever is closer to the intersection point is the actual node
                    if (!face_nodes.Contains(on_face)) face_nodes.Add(on_face);
                }

                Debug.Assert(face_nodes.Count == 3, "Three-Point-Intersection Evaluation failed.");

                // try to find a neighbor which has all three of the face_nodes as its own nodes.
                Dictionary<Tetrahedron, List<Node>> neighbors = intersected_tet.Key.neighbors;
                Tetrahedron matching_neighbor = null;

                // for each neighbor
                foreach (KeyValuePair<Tetrahedron, List<Node>> neighbor in neighbors)
                {
                    // if this tetrahedron even shares 3 nodes with the neighbor
                    if (neighbor.Value.Count == 3)
                    {
                        // check if these three nodes are the nodes we are looking for
                        for (int i = 0; i < 3; i++)
                        {
                            if (!neighbor.Value.Contains(face_nodes[i])) break;
                            matching_neighbor = neighbor.Key;
                        }
                    }
                    if (matching_neighbor != null) break;
                }

                // if we found a neighbor that is attached to the coplanar face
                if (matching_neighbor != null)
                {
                    // now we have to find out, which tetrahedron is in which halfspace of the fracture plane
                    Node[] nodes_of_t = intersected_tet.Key.nodes;

                    // for that we just find the one node which isn't on the fracture plane and check if it is in the positive or negative halfspace
                    Node not_on_face_t1 = null;
                    for (int i = 0; i < 4; i++)
                    {
                        if (!face_nodes.Contains(nodes_of_t[i])) not_on_face_t1 = nodes_of_t[i];
                    }
                    Debug.Assert(not_on_face_t1 != null, "Didn't find the Node which is not on the coplanar fracture face.");

                    nodes_of_t = matching_neighbor.nodes;
                    Node not_on_face_t2 = null;
                    for (int i = 0; i < 4; i++)
                    {
                        if (!face_nodes.Contains(nodes_of_t[i])) not_on_face_t2 = nodes_of_t[i];
                    }
                    Debug.Assert(not_on_face_t2 != null, "Didn't find the Node which is not on the coplanar fracture face.");

                    // reassign kZeroMinus and kZeroPlus accordingly.
                    if (MathUtility.IsOnPositiveSide(not_on_face_t1, Vector<float>.Build.DenseOfArray(new float[] { transform.position.x, transform.position.y, transform.position.z }), fracture_plane_normal))
                    {
                        intersected_tet.Key.nodes[MathUtility.GetIndexOf(intersected_tet.Key, not_on_face_t1)] = kZeroPlus;
                        matching_neighbor.nodes[MathUtility.GetIndexOf(matching_neighbor, not_on_face_t2)] = kZeroMinus;
                    }
                    else
                    {
                        intersected_tet.Key.nodes[MathUtility.GetIndexOf(intersected_tet.Key, not_on_face_t1)] = kZeroMinus;
                        matching_neighbor.nodes[MathUtility.GetIndexOf(matching_neighbor, not_on_face_t2)] = kZeroPlus;
                    }
                }
            }
            else // is not coplanar to face but cuts an edge
            {
                Debug.Assert(false, "This is not yet implemented, try to adapt the test case.");
                // find the nodes on the edge that lies within the fracture plane
                // find the nodes that are not on the edge
                Node node_on_intersection_edge = null;
                bool other_node_found = false;
                foreach (Node n in intersected_tet.Key.nodes)
                {
                    if (n.Equals(this)) continue;
                    foreach (Tuple<Node, Node, Vector<float>> intersec in intersected_tet.Value.Item2)
                    {
                        if (MathUtility.EqualsRoughly(MathUtility.ToVector(n.transform.position), intersec.Item3, 0.001f))
                        {
                            node_on_intersection_edge = n;
                            other_node_found = true;
                            break;
                        }
                    }
                }
                //Debug.Assert(node_on_intersection_edge != null, "didn't find the other node on the intersection line.");

                // TODO ...
            }
        }

        // for each two point intersected tetrahedron:
        foreach (KeyValuePair<Tetrahedron, List<Tuple<Node, Node, Vector<float>>>> intersected_tet in two_point_intersected_tets)
        {
            // we know for sure that the current tetrahedron will not exist afterwards, so we mark it as "old".
            old_tetrahedra.Add(intersected_tet.Key);

            // find out which node is on which side of the fracture plane
            List<Tuple<Node, bool>> nodes_with_halfspace_flag = new List<Tuple<Node, bool>>(); // true == positive halfspace
            Node[] nodes_of_t = intersected_tet.Key.nodes;
            int count_of_negative_nodes = 0;
            int count_of_positive_nodes = 0;

            for (int i = 0; i < 4; i++)
            {
                if (nodes_of_t[i].Equals(this)) continue;

                if (MathUtility.IsOnPositiveSide(nodes_of_t[i], world_pos_of_node, fracture_plane_normal))
                {
                    nodes_with_halfspace_flag.Add(new Tuple<Node, bool>(nodes_of_t[i], true));
                    count_of_positive_nodes++;
                }
                else
                {
                    nodes_with_halfspace_flag.Add(new Tuple<Node, bool>(nodes_of_t[i], false));
                    count_of_negative_nodes++;
                }
            }

            // identify k1, k2, k3
            Node kOne = null;
            Node kTwo;
            Node kThree;
            if (count_of_positive_nodes == 1)
            {
                foreach (Tuple<Node, bool> entry in nodes_with_halfspace_flag)
                {
                    if (entry.Item2)
                    {
                        kOne = entry.Item1;
                        nodes_with_halfspace_flag.Remove(entry);
                        break;
                    }
                }
            }
            else if (count_of_negative_nodes == 1)
            {
                foreach (Tuple<Node, bool> entry in nodes_with_halfspace_flag)
                {
                    if (!entry.Item2)
                    {
                        kOne = entry.Item1;
                        nodes_with_halfspace_flag.Remove(entry);
                        break;
                    }
                }
            }

            kTwo = nodes_with_halfspace_flag[0].Item1;
            kThree = nodes_with_halfspace_flag[1].Item1;

            // build three new tetrahedra
            Vector<float> kFourMatPos = world_pos_of_cube - intersected_tet.Value[0].Item3;
            Vector<float> kFiveMatPos = world_pos_of_cube - intersected_tet.Value[1].Item3;
            Vector<float> kFourWorldPos = intersected_tet.Value[0].Item3;
            Vector<float> kFiveWorldPos = intersected_tet.Value[1].Item3;

            GameObject kFour_go = Instantiate(node_prefab.gameObject, transform.parent);
            kFour_go.transform.localPosition = new Vector3(kFourMatPos.At(0), kFourMatPos.At(1), kFourMatPos.At(2));
            kFour_go.transform.position = new Vector3(kFourWorldPos.At(0), kFourWorldPos.At(1), kFourWorldPos.At(2));
            GameObject kFive_go = Instantiate(node_prefab.gameObject, transform.parent);
            kFive_go.transform.localPosition = new Vector3(kFiveMatPos.At(0), kFiveMatPos.At(1), kFiveMatPos.At(2));
            kFive_go.transform.position = new Vector3(kFiveWorldPos.At(0), kFiveWorldPos.At(1), kFiveWorldPos.At(2));
            new_nodes.Add(kFour_go.GetComponent<Node>());
            new_nodes.Add(kFive_go.GetComponent<Node>());

            // find one of the edges that cut the only quadriliteral face of the resulting polygon.
            // say 
            //      kTwo / kThree = w / y
            //      kFour / kFive = a / b
            // 
            // we define two edges
            //      (a - w) & (b - w) == (kFour - kTwo) & (kFive - kTwo)
            // 
            // one of these edges is cutting the quadriliteral face, one is not.
            // the one that is cutting it has a specific property:
            // 
            // say a - w is the cutting edge, then if we project both b and y on the edge, 
            // it will result in two vectors that both point in exactly opposite directions.
            // for b - w both projections will point in the same direction.

            Vector<float> w_to_a = MathUtility.ToVector(kTwo.transform.position) - MathUtility.ToVector(kFour_go.GetComponent<Node>().transform.position);
            Vector<float> w_to_b = MathUtility.ToVector(kTwo.transform.position) - MathUtility.ToVector(kFive_go.GetComponent<Node>().transform.position);
            Vector<float> w_to_y = MathUtility.ToVector(kTwo.transform.position) - MathUtility.ToVector(kThree.transform.position);

            // project w_to_b onto w_to_a
            Vector<float> tbc_1 = w_to_b - ((w_to_b.DotProduct(w_to_a) / (float)w_to_a.L2Norm()) * w_to_a);

            // project w_to_y onto w_to_a
            Vector<float> tbc_2 = w_to_y - ((w_to_y.DotProduct(w_to_a) / (float)w_to_a.L2Norm()) * w_to_a);

            Tetrahedron t1;
            Tetrahedron t2;
            Tetrahedron t3;

            if (count_of_positive_nodes == 1)
            {
                t1 = tet_builder.GenerateTetrahedron(new GameObject[] { kZeroPlus_go, kOne.gameObject, kFour_go, kFive_go }).GetComponent<Tetrahedron>();
                t2 = tet_builder.GenerateTetrahedron(new GameObject[] { kZeroMinus_go, kTwo.gameObject, kFour_go, kFive_go }).GetComponent<Tetrahedron>();

                if (tbc_1.DotProduct(tbc_2) < 0.0f)
                {
                    t3 = tet_builder.GenerateTetrahedron(new GameObject[] { kZeroMinus_go, kTwo.gameObject, kThree.gameObject, kFour_go }).GetComponent<Tetrahedron>();
                }
                else
                {
                    t3 = tet_builder.GenerateTetrahedron(new GameObject[] { kZeroMinus_go, kTwo.gameObject, kThree.gameObject, kFive_go }).GetComponent<Tetrahedron>();
                }
            }
            else
            {
                t1 = tet_builder.GenerateTetrahedron(new GameObject[] { kZeroMinus_go, kOne.gameObject, kFour_go, kFive_go }).GetComponent<Tetrahedron>();
                t2 = tet_builder.GenerateTetrahedron(new GameObject[] { kZeroPlus_go, kTwo.gameObject, kFour_go, kFive_go }).GetComponent<Tetrahedron>();

                if (tbc_1.DotProduct(tbc_2) < 0.0f)
                {
                    t3 = tet_builder.GenerateTetrahedron(new GameObject[] { kZeroPlus_go, kTwo.gameObject, kThree.gameObject, kFour_go }).GetComponent<Tetrahedron>();
                }
                else
                {
                    t3 = tet_builder.GenerateTetrahedron(new GameObject[] { kZeroPlus_go, kTwo.gameObject, kThree.gameObject, kFive_go }).GetComponent<Tetrahedron>();
                }
            }

            new_tetrahedra.Add(t1);
            new_tetrahedra.Add(t2);
            new_tetrahedra.Add(t3);

            Node first_intersected_edge_n1 = intersected_tet.Value[0].Item1;
            Node first_intersected_edge_n2 = intersected_tet.Value[0].Item2;
            Node second_intersected_edge_n1 = intersected_tet.Value[1].Item1;
            Node second_intersected_edge_n2 = intersected_tet.Value[1].Item2;

            // for each neighbor of the intersected tetrahedron
            foreach (KeyValuePair<Tetrahedron, List<Node>> neighbor in intersected_tet.Key.neighbors)
            {
                bool first_connected_edge_intersected = false;
                bool second_connected_edge_intersected = false;

                if (neighbor.Value.Contains(first_intersected_edge_n1) && neighbor.Value.Contains(first_intersected_edge_n2))
                {
                    first_connected_edge_intersected = true;
                }
                if (neighbor.Value.Contains(second_intersected_edge_n1) && neighbor.Value.Contains(second_intersected_edge_n2))
                {
                    second_connected_edge_intersected = true;
                }

                if (first_connected_edge_intersected || second_connected_edge_intersected)
                {
                    old_tetrahedra.Add(neighbor.Key);
                }

                if (first_connected_edge_intersected ^ second_connected_edge_intersected)
                {
                    Node w = null, x = null, y = null, z = null; // x & y are the nodes attached to the intersected edges, w & z are the other ones.

                    // detect n1 and n2
                    if (first_connected_edge_intersected && !second_connected_edge_intersected)
                    {
                        x = first_intersected_edge_n1;
                        y = first_intersected_edge_n2;
                    }

                    else if (second_connected_edge_intersected && !first_connected_edge_intersected)
                    {
                        x = second_intersected_edge_n1;
                        y = second_intersected_edge_n2;
                    }
                    Node a = MathUtility.WhichNodeIsOnLine(x, y, kFour_go.GetComponent<Node>(), kFive_go.GetComponent<Node>()); // new node at the intersection point

                    // detect n3 and n4
                    bool first_found = false;
                    foreach (Node n in neighbor.Key.nodes)
                    {
                        if (!n.Equals(x) && !n.Equals(y))
                        {
                            w = n;

                            if (first_found)
                            {
                                z = n;
                                break;
                            }

                            first_found = true;
                        }
                    }

                    Tetrahedron n_t1 = tet_builder.GenerateTetrahedron(new GameObject[] { a.gameObject, x.gameObject, w.gameObject, z.gameObject}).GetComponent<Tetrahedron>();
                    Tetrahedron n_t2 = tet_builder.GenerateTetrahedron(new GameObject[] { a.gameObject, y.gameObject, w.gameObject, z.gameObject }).GetComponent<Tetrahedron>();
                    new_tetrahedra.Add(n_t1);
                    new_tetrahedra.Add(n_t2);
                }

                if (first_connected_edge_intersected && second_connected_edge_intersected)
                {
                    Node x = null; // this is the node which is shared by both intersected edges
                    Node a = null, b = null; // these are the nodes which are at the intersection points
                    Node z = null; // this is the only node which is not attached to any of the intersected edges
                    Node w = null, y = null; // these are the other nodes which define the endpoint of the intersected edges but are not x.

                    if (first_intersected_edge_n1 == second_intersected_edge_n1)
                    {
                        x = first_intersected_edge_n1;
                        w = first_intersected_edge_n2;
                        y = second_intersected_edge_n2;
                    }
                    if (first_intersected_edge_n2 == second_intersected_edge_n1)
                    {
                        x = first_intersected_edge_n2;
                        w = first_intersected_edge_n1;
                        y = second_intersected_edge_n2;
                    }
                    if (first_intersected_edge_n1 == second_intersected_edge_n2)
                    {
                        x = first_intersected_edge_n1;
                        w = first_intersected_edge_n2;
                        y = second_intersected_edge_n1;
                    }
                    if (first_intersected_edge_n2 == second_intersected_edge_n2)
                    {
                        x = first_intersected_edge_n2;
                        w = first_intersected_edge_n1;
                        y = second_intersected_edge_n1;
                    }

                    a = MathUtility.WhichNodeIsOnLine(x, w, kFour_go.GetComponent<Node>(), kFive_go.GetComponent<Node>());
                    b = MathUtility.WhichNodeIsOnLine(x, y, kFour_go.GetComponent<Node>(), kFive_go.GetComponent<Node>());

                    foreach (Node n in neighbor.Key.nodes)
                    {
                        if (!n.Equals(x) && !n.Equals(y) && !n.Equals(w))
                        {
                            z = n;
                        }
                    }

                    Tetrahedron n_t1 = tet_builder.GenerateTetrahedron(new GameObject[] { a.gameObject, b.gameObject, x.gameObject, z.gameObject }).GetComponent<Tetrahedron>();

                    Vector<float> n_w_to_a = MathUtility.ToVector(a.transform.position) - MathUtility.ToVector(w.transform.position);
                    Vector<float> n_w_to_b = MathUtility.ToVector(b.transform.position) - MathUtility.ToVector(w.transform.position);
                    Vector<float> n_w_to_y = MathUtility.ToVector(y.transform.position) - MathUtility.ToVector(w.transform.position);

                    Vector<float> n_tbc_1 = n_w_to_b - ((n_w_to_b.DotProduct(n_w_to_a) / (float)n_w_to_a.L2Norm()) * n_w_to_a);
                    Vector<float> n_tbc_2 = n_w_to_y - ((n_w_to_y.DotProduct(n_w_to_a) / (float)n_w_to_a.L2Norm()) * n_w_to_a);

                    Tetrahedron n_t2 = tet_builder.GenerateTetrahedron(new GameObject[] { a.gameObject, b.gameObject, w.gameObject, z.gameObject }).GetComponent<Tetrahedron>();
                    Tetrahedron n_t3;

                    if (n_tbc_1.DotProduct(n_tbc_2) < 0.0f)
                    {
                        n_t3 = tet_builder.GenerateTetrahedron(new GameObject[] { a.gameObject, w.gameObject, y.gameObject, z.gameObject }).GetComponent<Tetrahedron>();
                    }
                    else
                    {
                        n_t3 = tet_builder.GenerateTetrahedron(new GameObject[] { b.gameObject, w.gameObject, y.gameObject, z.gameObject }).GetComponent<Tetrahedron>();
                    }

                    new_tetrahedra.Add(n_t1);
                    new_tetrahedra.Add(n_t2);
                    new_tetrahedra.Add(n_t3);
                }
            }
        }
        return new Tuple<List<Tetrahedron>, List<Tetrahedron>, List<Node>>(old_tetrahedra, new_tetrahedra, new_nodes);
    }

    public void AddTensileForce(Vector<float> tf)
    {
        tensile_forces.Add(tf);
    }

    public void AddCompressiveForce(Vector<float> cf)
    {
        compressive_forces.Add(cf);
    }

    public void ClearTensileAndCompressiveForces()
    {
        tensile_forces.Clear();
        compressive_forces.Clear();
    }

    public bool DoesCrackOccur(float toughness)
    {
        Debug.Assert(tensile_forces.Count == compressive_forces.Count);

        Vector<float> sumOverTensileForces = Vector<float>.Build.DenseOfArray(new float[] { 0.0f, 0.0f, 0.0f });
        Matrix<float> sumOfMOfTensileForces = Matrix<float>.Build.DenseOfRowArrays(new float[][]
        {
            new float[] { 0.0f, 0.0f, 0.0f },
            new float[] { 0.0f, 0.0f, 0.0f },
            new float[] { 0.0f, 0.0f, 0.0f }
        });
        foreach (Vector<float> tf in tensile_forces)
        {
            sumOverTensileForces += tf;
            sumOfMOfTensileForces += MathUtility.M(tf);
        }

        Vector<float> sumOverCompressiveForces = Vector<float>.Build.DenseOfArray(new float[] { 0.0f, 0.0f, 0.0f });
        Matrix<float> sumOfMOfCompressiveForces = Matrix<float>.Build.DenseOfRowArrays(new float[][]
        {
            new float[] { 0.0f, 0.0f, 0.0f },
            new float[] { 0.0f, 0.0f, 0.0f },
            new float[] { 0.0f, 0.0f, 0.0f }
        });
        foreach (Vector<float> cf in compressive_forces)
        {
            sumOverCompressiveForces += cf;
            sumOfMOfCompressiveForces += MathUtility.M(cf);
        }

        Matrix<float> st = (1 / 2.0f) * (-MathUtility.M(sumOverTensileForces) + sumOfMOfTensileForces + MathUtility.M(sumOverCompressiveForces) - sumOfMOfCompressiveForces);

        Debug.Assert(st.IsSymmetric(), "The fracture tensor is not symmetric, but it should be.");
        Evd<float> evd = st.Evd(Symmetricity.Symmetric);
        float max = Mathf.Max(Mathf.Max((float)evd.EigenValues.At(0).Real, (float)evd.EigenValues.At(1).Real), (float)evd.EigenValues.At(2).Real);

        if (toughness < max)
        {
            bool debug_worked = false;
            Vector<float> lhs_transformed_eigenvec_1 = st * evd.EigenVectors.Column(0);
            Vector<float> lhs_transformed_eigenvec_2 = st * evd.EigenVectors.Column(1);
            Vector<float> lhs_transformed_eigenvec_3 = st * evd.EigenVectors.Column(2);

            Vector<float> rhs_transformed_eigenvec_1 = max * evd.EigenVectors.Column(0);
            Vector<float> rhs_transformed_eigenvec_2 = max * evd.EigenVectors.Column(1);
            Vector<float> rhs_transformed_eigenvec_3 = max * evd.EigenVectors.Column(2);

            if (MathUtility.EqualsRoughly(lhs_transformed_eigenvec_1, rhs_transformed_eigenvec_1, maximum_difference: 0.1f))
            {
                fracture_plane_normal = evd.EigenVectors.Column(0) / (float)evd.EigenVectors.Column(0).L2Norm();
                debug_worked = true;
            }

            else if (MathUtility.EqualsRoughly(lhs_transformed_eigenvec_2, rhs_transformed_eigenvec_2, maximum_difference: 0.1f))
            {
                fracture_plane_normal = evd.EigenVectors.Column(1) / (float)evd.EigenVectors.Column(1).L2Norm();
                debug_worked = true;
            }

            else if (MathUtility.EqualsRoughly(lhs_transformed_eigenvec_3, rhs_transformed_eigenvec_3, maximum_difference: 0.1f))
            {
                fracture_plane_normal = evd.EigenVectors.Column(2) / (float)evd.EigenVectors.Column(2).L2Norm();
                debug_worked = true;
            }
            Debug.Assert(debug_worked);
            return true;
        }

        return false;
    }
}
