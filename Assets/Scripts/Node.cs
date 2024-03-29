﻿using System;
using UnityEngine;
using System.Collections.Generic;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Factorization;

public class Node : MonoBehaviour
{
    [SerializeField]
    private Transform node_prefab;

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

    /* This function remeshes all tetrahedra attached to *this Node.
     * returns a List of all old tetrahedra, which are to be removed, a List of all new tetrahedra which
    are to be added to the list of all tetrahedra and a List of new Nodes which are to be added
    to the list of all nodes. */
    public Tuple<List<Tetrahedron>, List<Tetrahedron>, List<Node>> Remesh(Vector3 world_position_of_simulated_object)
    {
        Vector<float> world_pos_of_node = Vector<float>.Build.DenseOfArray(new float[] 
        { 
            transform.position.x, 
            transform.position.y, 
            transform.position.z 
        });

        Vector<float> world_pos_of_cube = Vector<float>.Build.DenseOfArray(new float[] 
        { 
            world_position_of_simulated_object.x, 
            world_position_of_simulated_object.y, 
            world_position_of_simulated_object.z 
        });

        TetrahedronBuilder tet_builder = GameObject.Find("FEM_Mesh").GetComponent<TetrahedronBuilder>();

        // for each two-point-intersected tetrahedron, this dictionary holds a list of Tuples, where one Tuple describes the edge (item1 - item2) and the world_pos (item3) of the intersection
        Dictionary<Tetrahedron, List<Tuple<Node, Node, Vector<float>>>> two_point_intersected_tets = new Dictionary<Tetrahedron, List<Tuple<Node, Node, Vector<float>>>>();

        // for each coplanar-intersected tetrahedron, this dictionary holds a List of Nodes which define the face of the tetrahedron the fracture plane is coplanar to.
        Dictionary<Tetrahedron, List<Node>> coplanar_intersected_tets = new Dictionary<Tetrahedron, List<Node>>();

        // for each parallel-intersected tetrahedron, this dictionary holds a List of Tuples where one Tuple describes an intersection by 
        // the edge (item1 - item2) and the world_pos (item3) of the intersection
        Dictionary<Tetrahedron, List<Tuple<Node, Node, Vector<float>>>> parallel_edge_but_not_coplanar_intersected_tets = new Dictionary<Tetrahedron, List<Tuple<Node, Node, Vector<float>>>>();

        // and there is a list for each non-intersected tetrahedron.
        List<Tetrahedron> not_intersected_tets = new List<Tetrahedron>();

        // we also collect old tetrahedra, which are going to be removed
        List<Tetrahedron> old_tetrahedra = new List<Tetrahedron>();

        // we collect new tetrahedra, which are going to be added to the list of all tetrahedra
        List<Tetrahedron> new_tetrahedra = new List<Tetrahedron>();

        // and we collect newly created nodes, which are going to be added to the list of all nodes
        List<Node> new_nodes = new List<Node>();

        foreach (Tetrahedron t in attached_elements)
        {
            // in here, we want to test each tetrahedron attached to this node for intersection with the
            // fracture plane. This can result in different cases, which we all need to treat differently.

            //      - no intersection
            //      - fracture plane is coplanar to a face of the tetrahedron
            //      - one edge of the tetrahedron lies inside the fracture plane
            //      - the fracture plane cuts right through the tetrahedron

            // we can distinguish these cases by testing which edge of the tetrahedron intersects with the fracture plane.

            Tuple<bool /*is there an edge of the tet that is parallel to the fracture plane but the fracture plane is NOT coplanar with a face of the tet*/, 
                bool /*is the fracture plane coplanar with a face*/, 
                List<Tuple<Node, Node, Vector<float>>>> intersection_data = t.Intersect(this, fracture_plane_normal);
            List<Tuple<Node, Node, Vector<float>>> intersection_points = intersection_data.Item3;

            if(intersection_data.Item1)
            {
                parallel_edge_but_not_coplanar_intersected_tets.Add(t, intersection_points);
            }
            if(intersection_data.Item2)
            {
                foreach(Tuple<Node, Node, Vector<float>> intersection_point in intersection_points)
                {
                    if(intersection_point.Item3 == null)
                    {
                        List<Node> temp = new List<Node>();
                        temp.Add(this);
                        temp.Add(intersection_point.Item1);
                        temp.Add(intersection_point.Item2);
                        coplanar_intersected_tets.Add(t, temp);
                        break;
                    }
                }
            }

            if(!intersection_data.Item1 && !intersection_data.Item2 && intersection_data.Item3.Count != 0)
            {
                two_point_intersected_tets.Add(t, intersection_points);
                Debug.Assert(intersection_data.Item3.Count == 2);
            }
            else if(!intersection_data.Item1 && !intersection_data.Item2 && intersection_data.Item3.Count == 0)
            {
                not_intersected_tets.Add(t);
            }
        }

        // *This Node is exceeding its deformation limit. This means that it will be destroyed and
        // replaced with two new nodes at the same position. one of these nodes is in the positive halfspace of the 
        // fracture plane, one is in the negative halfspace.

        GameObject kZeroPlus_go = Instantiate(node_prefab.gameObject, transform.parent);
        kZeroPlus_go.transform.position = transform.position;
        kZeroPlus_go.transform.localPosition = transform.localPosition;

        GameObject kZeroMinus_go = Instantiate(node_prefab.gameObject, transform.parent);
        kZeroMinus_go.transform.position = transform.position;
        kZeroMinus_go.transform.localPosition = transform.localPosition;

        Node kZeroPlus = kZeroPlus_go.GetComponent<Node>();
        Node kZeroMinus = kZeroMinus_go.GetComponent<Node>();

        new_nodes.Add(kZeroPlus);
        new_nodes.Add(kZeroMinus);

        // some debug information
        // ======================================================================================================================================
        Debug.Log("Crack-Info: There are " + two_point_intersected_tets.Count + " cases of two_point-intersections, " 
            + coplanar_intersected_tets.Count + " cases of coplanar-intersections, " 
            + parallel_edge_but_not_coplanar_intersected_tets.Count + " cases of parallel_edge_but_not_coplanar-intersections and "
            + not_intersected_tets.Count + " cases of non-intersected tetrahedra.");
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

        // it is possible for a crack to not hit any tetrahedron, so we have to account for that.
        if (not_intersected_tets.Count > 0 && two_point_intersected_tets.Count == 0 && coplanar_intersected_tets.Count == 0 && parallel_edge_but_not_coplanar_intersected_tets.Count == 0)
        {
            if(kZeroMinus.attached_elements.Count == 0)
            {
                Destroy(kZeroMinus_go);
                new_nodes.Remove(kZeroMinus);
            }
            if (kZeroPlus.attached_elements.Count == 0)
            {
                Destroy(kZeroPlus_go);
                new_nodes.Remove(kZeroPlus);
            }
            return new Tuple<List<Tetrahedron>, List<Tetrahedron>, List<Node>>(old_tetrahedra, new_tetrahedra, new_nodes);
        }

        // for each coplanar intersection:
        foreach (KeyValuePair<Tetrahedron, List<Node>> coplanar_tet in coplanar_intersected_tets)
        {
            // this is a unique setting: if we have a neighbor at the face which is coplanar, this neighbor is also in the list of the coplanar_intersected_tets.
            // so we have to account for that by removing the neighbor from the list.
            if(!coplanar_tet.Key.node_transforms.Contains(this.transform))
            {
                continue;
            }

            // first collect the nodes which are on the fracture plane
            List<Node> face_nodes = coplanar_tet.Value;

            // try to find a neighbor which has all three of the face_nodes as its own nodes.
            Dictionary<Tetrahedron, List<Node>> neighbors = coplanar_tet.Key.neighbors;
            Tetrahedron matching_neighbor = null;

            List<Vector<float>> this_node_positions = new List<Vector<float>>();
            this_node_positions.Add(Vector<float>.Build.DenseOfArray(new float[] { coplanar_tet.Key.nodes[0].transform.position.x, coplanar_tet.Key.nodes[0].transform.position.y, coplanar_tet.Key.nodes[0].transform.position.z }));
            this_node_positions.Add(Vector<float>.Build.DenseOfArray(new float[] { coplanar_tet.Key.nodes[1].transform.position.x, coplanar_tet.Key.nodes[1].transform.position.y, coplanar_tet.Key.nodes[1].transform.position.z }));
            this_node_positions.Add(Vector<float>.Build.DenseOfArray(new float[] { coplanar_tet.Key.nodes[2].transform.position.x, coplanar_tet.Key.nodes[2].transform.position.y, coplanar_tet.Key.nodes[2].transform.position.z }));
            this_node_positions.Add(Vector<float>.Build.DenseOfArray(new float[] { coplanar_tet.Key.nodes[3].transform.position.x, coplanar_tet.Key.nodes[3].transform.position.y, coplanar_tet.Key.nodes[3].transform.position.z }));

            foreach (KeyValuePair<Tetrahedron, List<Node>> neighbor in neighbors)
            {
                List<Vector<float>> neighbor_node_positions = new List<Vector<float>>();
                neighbor_node_positions.Add(Vector<float>.Build.DenseOfArray(new float[] { neighbor.Key.nodes[0].transform.position.x, neighbor.Key.nodes[0].transform.position.y, neighbor.Key.nodes[0].transform.position.z }));
                neighbor_node_positions.Add(Vector<float>.Build.DenseOfArray(new float[] { neighbor.Key.nodes[1].transform.position.x, neighbor.Key.nodes[1].transform.position.y, neighbor.Key.nodes[1].transform.position.z }));
                neighbor_node_positions.Add(Vector<float>.Build.DenseOfArray(new float[] { neighbor.Key.nodes[2].transform.position.x, neighbor.Key.nodes[2].transform.position.y, neighbor.Key.nodes[2].transform.position.z }));
                neighbor_node_positions.Add(Vector<float>.Build.DenseOfArray(new float[] { neighbor.Key.nodes[3].transform.position.x, neighbor.Key.nodes[3].transform.position.y, neighbor.Key.nodes[3].transform.position.z }));

                List<Vector<float>> connections = new List<Vector<float>>();
                for(int i = 0; i < 4; i++)
                {
                    for (int j = 0; j < 4; j++)
                    {
                        if(MathUtility.EqualsRoughly(this_node_positions[i], neighbor_node_positions[j], 0.0f))
                        {
                            connections.Add(this_node_positions[i]);
                        }
                    }
                }

                if(connections.Count == 3)
                {
                    matching_neighbor = neighbor.Key;
                }
            }

            // if we found a neighbor that is attached to the coplanar face
            if (matching_neighbor != null)
            {
                // now we have to find out, which tetrahedron is in which halfspace of the fracture plane
                Node[] nodes_of_t = coplanar_tet.Key.nodes;

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
                    coplanar_tet.Key.nodes[MathUtility.GetIndexOf(coplanar_tet.Key, this)] = kZeroPlus;
                    matching_neighbor.nodes[MathUtility.GetIndexOf(matching_neighbor, this)] = kZeroMinus;

                    int index = coplanar_tet.Key.node_transforms.IndexOf(this.transform);
                    coplanar_tet.Key.node_transforms.RemoveAt(index);
                    coplanar_tet.Key.node_transforms.Insert(index, kZeroPlus.transform);

                    index = matching_neighbor.node_transforms.IndexOf(this.transform);
                    matching_neighbor.node_transforms.RemoveAt(index);
                    matching_neighbor.node_transforms.Insert(index, kZeroMinus.transform);
                }
                else
                {
                    coplanar_tet.Key.nodes[MathUtility.GetIndexOf(coplanar_tet.Key, this)] = kZeroMinus;
                    matching_neighbor.nodes[MathUtility.GetIndexOf(matching_neighbor, this)] = kZeroPlus;

                    int index = coplanar_tet.Key.node_transforms.IndexOf(this.transform);
                    coplanar_tet.Key.node_transforms.RemoveAt(index);
                    coplanar_tet.Key.node_transforms.Insert(index, kZeroMinus.transform);

                    index = matching_neighbor.node_transforms.IndexOf(this.transform);
                    matching_neighbor.node_transforms.RemoveAt(index);
                    matching_neighbor.node_transforms.Insert(index, kZeroPlus.transform);
                }
            }
        }

        foreach(KeyValuePair<Tetrahedron, List<Tuple<Node, Node, Vector<float>>>> intersected_tet in parallel_edge_but_not_coplanar_intersected_tets)
        {
            // we have to deal with 2 possible cases:
            // first case is that the fracture plane only touches the tetrahedron at the exact edge which it is parallel to. 
            // if this happens, we barely have to do anything.
            // second case is that the fracture plane intersects with exactly one other edge. remeshing is required but still pretty simple.
            // it is not an option that it is parallel to 2 edges, because then it would count as the fracture plane being coplanar to one face of the tetrahedron.

            if(parallel_edge_but_not_coplanar_intersected_tets.Count == 2) // there is no other intersection
            {
                // we basically treat this case like a non-intersected tetrahedron
                // determine in which halfspace of the fracture plane this tetrahedron is and reassign the cracked node (this)
                // accordingly to kZeroPlus or kZeroMinus

                // find node not on the parallel edge. there are two nodes of interest, both appear only once in the list of intersected edges
                Node x = null;
                for (int i = 0; i < 4; i++)
                {
                    int occurrences = 0;
                    x = intersected_tet.Key.nodes[i];

                    if (this.Equals(x)) continue;
                    if (intersected_tet.Value[0].Item1.Equals(x))
                    {
                        occurrences++;
                    }
                    if(intersected_tet.Value[0].Item2.Equals(x))
                    {
                        occurrences++;
                    }
                    if(intersected_tet.Value[1].Item1.Equals(x))
                    {
                        occurrences++;
                    }
                    if(intersected_tet.Value[1].Item2.Equals(x))
                    {
                        occurrences++;
                    }
                    if (occurrences == 1)
                    {
                        break;
                    }
                    else
                    {
                        x = null;
                    }
                }

                Debug.Assert(x != null);

                if(MathUtility.IsOnPositiveSide(x, MathUtility.ToVector(this.transform.position), this.fracture_plane_normal))
                {
                    intersected_tet.Key.SwapNodes(this, kZeroPlus);
                }
                else
                {
                    intersected_tet.Key.SwapNodes(this, kZeroMinus);
                }
            }
            else if(parallel_edge_but_not_coplanar_intersected_tets.Count == 3) // there is one other intersection
            {
                Tuple<Node, Node, Vector<float>> intersection_of_interest = null;
                // find the one intersection case that has a non-null Vector as the intersection point
                foreach (Tuple<Node, Node, Vector<float>> intersection in intersected_tet.Value)
                {
                    if (intersection.Item3 != null)
                    {
                        intersection_of_interest = intersection;
                    }
                }
                Debug.Assert(intersection_of_interest != null);

                // separate the tetrahedron in two tetrahedra and remesh the potential neighbors at the intersected edge accordingly.
                Vector<float> kMatPos = world_pos_of_cube - intersection_of_interest.Item3;
                Vector<float> kWorldPos = intersection_of_interest.Item3;

                GameObject k_go = Instantiate(node_prefab.gameObject, transform.parent);
                k_go.transform.localPosition = new Vector3(kMatPos.At(0), kMatPos.At(1), kMatPos.At(2));
                k_go.transform.position = new Vector3(kWorldPos.At(0), kWorldPos.At(1), kWorldPos.At(2));

                Node k = k_go.GetComponent<Node>();
                k.old_world_position = new Vector3(kWorldPos.At(0), kWorldPos.At(1), kWorldPos.At(2));
                new_nodes.Add(k);

                Tetrahedron t1;
                Tetrahedron t2;

                Node x = intersection_of_interest.Item1;
                Node y = intersection_of_interest.Item2;

                // find the node that is not *this, x, or y.
                Tetrahedron t = intersected_tet.Key;

                int correct_index = -1;
                for (int i = 0; i < 4; i++)
                {
                    if (!(MathUtility.GetIndexOf(t, x) == i || MathUtility.GetIndexOf(t, y) == i || MathUtility.GetIndexOf(t, this) == i))
                    {
                        correct_index = i;
                    }
                }

                Debug.Assert(correct_index != -1);
                Node z = t.nodes[correct_index];

                bool x_is_on_pos_side = MathUtility.IsOnPositiveSide(x, world_pos_of_node, fracture_plane_normal);
                if (x_is_on_pos_side)
                {
                    t1 = tet_builder.GenerateTetrahedron(new GameObject[] { kZeroPlus_go, x.gameObject, k_go, z.gameObject }).GetComponent<Tetrahedron>();
                    t2 = tet_builder.GenerateTetrahedron(new GameObject[] { kZeroMinus_go, y.gameObject, k_go, z.gameObject }).GetComponent<Tetrahedron>();
                }
                else
                {
                    t1 = tet_builder.GenerateTetrahedron(new GameObject[] { kZeroPlus_go, y.gameObject, k_go, z.gameObject }).GetComponent<Tetrahedron>();
                    t2 = tet_builder.GenerateTetrahedron(new GameObject[] { kZeroMinus_go, x.gameObject, k_go, z.gameObject }).GetComponent<Tetrahedron>();
                }

                old_tetrahedra.Add(t);
                new_tetrahedra.Add(t1);
                new_tetrahedra.Add(t2);

                // are there neighbors which share the intersected edge?
                List<Tuple<Tetrahedron, List<Node>>> matching_neighbors = new List<Tuple<Tetrahedron, List<Node>>>();

                foreach (KeyValuePair<Tetrahedron, List<Node>> neighbor in intersected_tet.Key.neighbors)
                {
                    if (neighbor.Value.Contains(x) && neighbor.Value.Contains(y))
                    {
                        matching_neighbors.Add(new Tuple<Tetrahedron, List<Node>>(neighbor.Key, neighbor.Value));
                    }
                }

                // remesh the neighbor
                foreach (Tuple<Tetrahedron, List<Node>> matching_n in matching_neighbors)
                {
                    if (matching_n.Item2.Count == 2 || (matching_n.Item2.Count == 3 && !matching_n.Item2.Contains(this)))
                    {
                        // find both nodes of matching_n that are not x and y
                        Node[] n_nodes = matching_n.Item1.nodes;
                        Node[] a_and_b = new Node[2];
                        int index = 0;

                        for (int i = 0; i < 4; i++)
                        {
                            if (!(i == MathUtility.GetIndexOf(matching_n.Item1, x) || i == MathUtility.GetIndexOf(matching_n.Item1, y)))
                            {
                                a_and_b[index] = n_nodes[i];
                                index++;
                            }
                        }

                        Tetrahedron t3 = tet_builder.GenerateTetrahedron(new GameObject[] { x.gameObject, k_go, a_and_b[0].gameObject, a_and_b[1].gameObject }).GetComponent<Tetrahedron>();
                        Tetrahedron t4 = tet_builder.GenerateTetrahedron(new GameObject[] { y.gameObject, k_go, a_and_b[0].gameObject, a_and_b[1].gameObject }).GetComponent<Tetrahedron>();

                        old_tetrahedra.Add(matching_n.Item1);
                        new_tetrahedra.Add(t3);
                        new_tetrahedra.Add(t4);
                    }
                }
            }
        }

        // for each two point intersected tetrahedron:
        foreach (KeyValuePair<Tetrahedron, List<Tuple<Node, Node, Vector<float>>>> intersected_tet in two_point_intersected_tets)
        {
            // if this tetrahedron has been marked as old by being remeshed as a neighbor of a previous tetrahedron,
            // we don't need to remesh it again here.
            if (old_tetrahedra.Contains(intersected_tet.Key))
            {
                continue;
            }
            else
            {
                // we know for sure that the current tetrahedron will not exist afterwards, so we mark it as "old".
                old_tetrahedra.Add(intersected_tet.Key);
            }

            // there is exactly one node of the tetrahedron, which is on one side of the fracture plane, and
            // two nodes which are on the other side of it. we have to find out which is where to remesh accordingly.
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

            // identify k1 which is the lonely node in one halfspace of the tetrahedron, 
            // identify k2 and k3 which are both in the other halfspace.
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

            Node kFour = kFour_go.GetComponent<Node>();
            kFour.old_world_position = new Vector3(kFourWorldPos.At(0), kFourWorldPos.At(1), kFourWorldPos.At(2));

            Node kFive = kFive_go.GetComponent<Node>();
            kFive.old_world_position = new Vector3(kFiveWorldPos.At(0), kFiveWorldPos.At(1), kFiveWorldPos.At(2));

            new_nodes.Add(kFour);
            new_nodes.Add(kFive);

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

            Vector<float> w_to_a = MathUtility.ToVector(kFour_go.GetComponent<Node>().transform.position) - MathUtility.ToVector(kTwo.transform.position);
            Vector<float> w_to_b = MathUtility.ToVector(kFive_go.GetComponent<Node>().transform.position) - MathUtility.ToVector(kTwo.transform.position);
            Vector<float> w_to_y = MathUtility.ToVector(kThree.transform.position) - MathUtility.ToVector(kTwo.transform.position);

            // project w_to_b onto w_to_a
            Vector<float> tbc_1 = MathUtility.ProjectOnto(w_to_b, w_to_a) - w_to_b;

            // project w_to_y onto w_to_a
            Vector<float> tbc_2 = MathUtility.ProjectOnto(w_to_y, w_to_a) - w_to_y;

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

                // if the neighbor has exactly one matching edge that is intersected
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
                            if (!first_found)
                            {
                                w = n;
                            }

                            if (first_found)
                            {
                                z = n;
                                break;
                            }

                            first_found = true;
                        }
                    }
                    Tetrahedron n_t1 = null;
                    Tetrahedron n_t2 = null;

                    if (w == this)
                    {
                        if (MathUtility.IsOnPositiveSide(x, world_pos_of_node, fracture_plane_normal))
                        {
                            n_t1 = tet_builder.GenerateTetrahedron(new GameObject[] { a.gameObject, x.gameObject, kZeroPlus_go, z.gameObject }).GetComponent<Tetrahedron>();
                            n_t2 = tet_builder.GenerateTetrahedron(new GameObject[] { a.gameObject, y.gameObject, kZeroMinus_go, z.gameObject }).GetComponent<Tetrahedron>();
                        }
                        else
                        {
                            n_t1 = tet_builder.GenerateTetrahedron(new GameObject[] { a.gameObject, x.gameObject, kZeroMinus_go, z.gameObject }).GetComponent<Tetrahedron>();
                            n_t2 = tet_builder.GenerateTetrahedron(new GameObject[] { a.gameObject, y.gameObject, kZeroPlus_go, z.gameObject }).GetComponent<Tetrahedron>();
                        }
                    }
                    else if (z == this)
                    {
                        if (MathUtility.IsOnPositiveSide(x, world_pos_of_node, fracture_plane_normal))
                        {
                            n_t1 = tet_builder.GenerateTetrahedron(new GameObject[] { a.gameObject, x.gameObject, w.gameObject, kZeroPlus_go }).GetComponent<Tetrahedron>();
                            n_t2 = tet_builder.GenerateTetrahedron(new GameObject[] { a.gameObject, y.gameObject, w.gameObject, kZeroMinus_go }).GetComponent<Tetrahedron>();
                        }
                        else
                        {
                            n_t1 = tet_builder.GenerateTetrahedron(new GameObject[] { a.gameObject, x.gameObject, w.gameObject, kZeroMinus_go }).GetComponent<Tetrahedron>();
                            n_t2 = tet_builder.GenerateTetrahedron(new GameObject[] { a.gameObject, y.gameObject, w.gameObject, kZeroPlus_go }).GetComponent<Tetrahedron>();
                        }
                    }
                    else
                    {
                        n_t1 = tet_builder.GenerateTetrahedron(new GameObject[] { a.gameObject, x.gameObject, w.gameObject, z.gameObject }).GetComponent<Tetrahedron>();
                        n_t2 = tet_builder.GenerateTetrahedron(new GameObject[] { a.gameObject, y.gameObject, w.gameObject, z.gameObject }).GetComponent<Tetrahedron>();
                    }

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

                    a = MathUtility.WhichNodeIsOnLine(x, y, kFour_go.GetComponent<Node>(), kFive_go.GetComponent<Node>());
                    b = MathUtility.WhichNodeIsOnLine(x, w, kFour_go.GetComponent<Node>(), kFive_go.GetComponent<Node>());

                    foreach (Node n in neighbor.Key.nodes)
                    {
                        if (!n.Equals(x) && !n.Equals(y) && !n.Equals(w))
                        {
                            z = n;
                        }
                    }

                    Debug.Assert(z != null, "the independent node was not found.");

                    Tetrahedron n_t1 = tet_builder.GenerateTetrahedron(new GameObject[] { a.gameObject, b.gameObject, x.gameObject, z.gameObject }).GetComponent<Tetrahedron>();
                    Tetrahedron n_t2 = tet_builder.GenerateTetrahedron(new GameObject[] { a.gameObject, b.gameObject, w.gameObject, z.gameObject }).GetComponent<Tetrahedron>();
                    Tetrahedron n_t3 = tet_builder.GenerateTetrahedron(new GameObject[] { a.gameObject, w.gameObject, y.gameObject, z.gameObject }).GetComponent<Tetrahedron>();

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

    public bool IsExceedingDeformationLimit(float toughness)
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

        // this is the separation tensor sigma
        Matrix<float> st = -MathUtility.M(sumOverTensileForces) + sumOfMOfTensileForces + MathUtility.M(sumOverCompressiveForces) - sumOfMOfCompressiveForces;
        st /= 1 / 2.0f;

        Debug.Assert(st.IsSymmetric(), "The fracture tensor is not symmetric, but it should be.");
        Evd<float> evd = st.Evd(Symmetricity.Symmetric);
        float max = Mathf.Max(Mathf.Max((float)evd.EigenValues.At(0).Real, (float)evd.EigenValues.At(1).Real), (float)evd.EigenValues.At(2).Real);

        if (toughness < max)
        {
            bool found_corresponding_eigenvector = false;
            Vector<float> lhs_transformed_eigenvec_1 = st * evd.EigenVectors.Column(0);
            Vector<float> lhs_transformed_eigenvec_2 = st * evd.EigenVectors.Column(1);
            Vector<float> lhs_transformed_eigenvec_3 = st * evd.EigenVectors.Column(2);

            Vector<float> rhs_transformed_eigenvec_1 = max * evd.EigenVectors.Column(0);
            Vector<float> rhs_transformed_eigenvec_2 = max * evd.EigenVectors.Column(1);
            Vector<float> rhs_transformed_eigenvec_3 = max * evd.EigenVectors.Column(2);

            if (MathUtility.EqualsRoughly(lhs_transformed_eigenvec_1, rhs_transformed_eigenvec_1, maximum_difference: 0.5f))
            {
                fracture_plane_normal = evd.EigenVectors.Column(0) / (float)evd.EigenVectors.Column(0).L2Norm();
                found_corresponding_eigenvector = true;
            }

            else if (MathUtility.EqualsRoughly(lhs_transformed_eigenvec_2, rhs_transformed_eigenvec_2, maximum_difference: 0.5f))
            {
                fracture_plane_normal = evd.EigenVectors.Column(1) / (float)evd.EigenVectors.Column(1).L2Norm();
                found_corresponding_eigenvector = true;
            }

            else if (MathUtility.EqualsRoughly(lhs_transformed_eigenvec_3, rhs_transformed_eigenvec_3, maximum_difference: 0.5f))
            {
                fracture_plane_normal = evd.EigenVectors.Column(2) / (float)evd.EigenVectors.Column(2).L2Norm();
                found_corresponding_eigenvector = true;
            }

            Debug.Assert(found_corresponding_eigenvector, "This Error is most likely here, because one Tetrahedron attached to a Node is so small," +
                "that it is impossible to correctly calculate the eigenvalues and eigenvectors.");

            return true;
        }

        return false;
    }
}
