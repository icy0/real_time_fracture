using System.Collections.Generic;
using System.Collections;
using System.Linq;
using System;
using UnityEngine;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Factorization;

public class MathUtility
{
    public static int KroneckerDelta(int i, int j)
    {
        if (i == j) return 1;
        else return 0;
    }

    public static bool IsOnPositiveSide(Node n, Vector<float> point_on_plane, Vector<float> plane_normal)
    {
        Vector<float> n_pos = Vector<float>.Build.DenseOfArray(new float[] { n.world_pos.x, n.world_pos.y, n.world_pos.z});

        if(plane_normal.DotProduct(n_pos - point_on_plane) > 0)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    public static Node WhichNodeIsOneLine(Node line_n1, Node line_n2, Node x, Node y)
    {
        // TODO

        return null;
    }

    public static int GetIndexOf(Tetrahedron x, Node n)
    {
        Node[] nodes = x.GetNodes();
        int index = -1;

        for(int i = 0; i < 4; i++)
        {
            if(nodes[i].Equals(n))
            {
                return index = i;
            }
        }

        return index;
    }

    public static Vector<float> KroneckerDeltaVector(int i)
    {
        Vector<float> kdv = Vector<float>.Build.DenseOfArray(new float[] { 0.0f, 0.0f, 0.0f, 0.0f });

        if (i == 1)
            kdv.At(0, 1.0f);
        else if (i == 2)
            kdv.At(1, 1.0f);
        else if (i == 3)
            kdv.At(2, 1.0f);

        return kdv;
    }

    public static Matrix<float> M(Vector<float> a)
    {
        if (!(a.At(0) == 0.0 && a.At(1) == 0.0 && a.At(2) == 0.0))
        {
            Matrix<float> m = Matrix<float>.Build.DenseOfRowArrays(new float[][]
            {
               new float[] { a.At(0) * a.At(0), a.At(0) * a.At(1), a.At(0) * a.At(2)},
               new float[] { a.At(1) * a.At(0), a.At(1) * a.At(1), a.At(1) * a.At(2)},
               new float[] { a.At(2) * a.At(0), a.At(2) * a.At(1), a.At(2) * a.At(2)}
            });

            return m / (float) a.L2Norm();
        }
        else
        {
            return Matrix<float>.Build.DenseOfRowArrays(new float[][]
            {
               new float[] { 0.0f, 0.0f, 0.0f },
               new float[] { 0.0f, 0.0f, 0.0f },
               new float[] { 0.0f, 0.0f, 0.0f }
            });
        }
    }
}

public class Node
{
    public Vector3 mat_pos;
    public Vector3 world_pos;
    public Vector3 new_world_pos;
    public Vector3 world_speed;

    public Vector<float> fracture_plane_normal;
    public List<Tetrahedron> attached_elements = new List<Tetrahedron>();

    public List<Vector<float>> tensile_forces = new List<Vector<float>>();
    public List<Vector<float>> compressive_forces = new List<Vector<float>>();

    public Node(Vector3 m_pos, Vector3 w_pos)
    {
        mat_pos = m_pos;
        world_pos = w_pos;
        new_world_pos = w_pos;
        world_speed = new Vector3(0.0f, 0.0f, 0.0f);
    }

    public void Crack(Vector3 world_position_of_cube)
    {
        Vector<float> world_pos_of_node = Vector<float>.Build.DenseOfArray(new float[] { world_pos.x, world_pos.y, world_pos.z });
        Vector<float> world_pos_of_cube = Vector<float>.Build.DenseOfArray(new float[] { world_position_of_cube.x, world_position_of_cube.y, world_position_of_cube.z });

        Node kZeroPlus = new Node(m_pos: mat_pos, w_pos: world_pos);
        Node kZeroMinus = new Node(m_pos: mat_pos, w_pos: world_pos);

        // for each intersected tetrahedron, this dictionary holds a list of Tuples, where one Tuple describes the edge (item1 - item2) and the world_pos (item3) of the intersection
        Dictionary<Tetrahedron, List<Tuple<Node, Node, Vector<float>>>> intersected_tets = new Dictionary<Tetrahedron, List<Tuple<Node, Node, Vector<float>>>>();
        List<Tetrahedron> not_intersected_tets = new List<Tetrahedron>();

        List<Tetrahedron> old_tetrahedra = new List<Tetrahedron>();
        List<Tetrahedron> new_tetrahedra = new List<Tetrahedron>();
        List<Node> new_nodes = new List<Node>();

        foreach (Tetrahedron t in attached_elements)
        {
            List<Tuple<Node, Node, Vector<float>>> intersection_points = new List<Tuple<Node, Node, Vector<float>>>();
            intersection_points = t.Intersect(world_pos_of_node, fracture_plane_normal);

            if (intersection_points.Count != 0)
            {
                intersected_tets.Add(t, intersection_points);
            }
            else
            {
                not_intersected_tets.Add(t);
            }
        }

        // for each non-hit tetrahedron:
        foreach (Tetrahedron not_hit_t in not_intersected_tets)
        {
            int index_of_n = MathUtility.GetIndexOf(not_hit_t, this);
            Debug.Assert(index_of_n != -1);

            if (MathUtility.IsOnPositiveSide(not_hit_t.GetNodes()[(index_of_n + 1) % 4], world_pos_of_node, fracture_plane_normal))
            {
                not_hit_t.SwapNodes(this, kZeroPlus);
            }
            else
            {
                not_hit_t.SwapNodes(this, kZeroMinus);
            }
        }

        // for each hit tetrahedron:
        foreach(KeyValuePair<Tetrahedron, List<Tuple<Node, Node, Vector<float>>>> intersected_tet in intersected_tets)
        {
            old_tetrahedra.Add(intersected_tet.Key);
            
            // find out which node is on which side of the fracture plane
            List<Tuple<Node, bool>> node_on_positive_side = new List<Tuple<Node, bool>>();
            Node[] nodes_of_t = intersected_tet.Key.GetNodes();
            int count_of_negative_nodes = 0;
            int count_of_positive_nodes = 0;

            for (int i = 0; i < 4; i++)
            {
                if (nodes_of_t[i].Equals(this)) continue;

                if (MathUtility.IsOnPositiveSide(nodes_of_t[i], world_pos_of_node, fracture_plane_normal))
                {
                    node_on_positive_side.Add(new Tuple<Node, bool>(nodes_of_t[i], true));
                    count_of_positive_nodes++;
                }
                else
                {
                    node_on_positive_side.Add(new Tuple<Node, bool>(nodes_of_t[i], false));
                    count_of_negative_nodes++;
                }
            }

            // identify k1, k2, k3
            Node kOne = null;
            Node kTwo;
            Node kThree;
            if(count_of_positive_nodes == 1)
            {
                foreach(Tuple<Node, bool> entry in node_on_positive_side)
                {
                    if (entry.Item2)
                    {
                        kOne = entry.Item1;
                        node_on_positive_side.Remove(entry);
                        break;
                    }
                }
            }
            else if(count_of_negative_nodes == 1)
            {
                foreach (Tuple<Node, bool> entry in node_on_positive_side)
                {
                    if (!entry.Item2)
                    {
                        kOne = entry.Item1;
                        node_on_positive_side.Remove(entry);
                        break;
                    }
                }
            }

            kTwo = node_on_positive_side[0].Item1;
            kThree = node_on_positive_side[1].Item1;

            // build three new tetrahedra
            Vector<float> kFourMatPos = world_pos_of_cube - intersected_tet.Value[0].Item3;
            Vector<float> kFiveMatPos = world_pos_of_cube - intersected_tet.Value[1].Item3;
            Vector<float> kFourWorldPos = intersected_tet.Value[0].Item3;
            Vector<float> kFiveWorldPos = intersected_tet.Value[1].Item3;
            Node kFour = new Node(new Vector3(kFourMatPos.At(0), kFourMatPos.At(1), kFourMatPos.At(2)), new Vector3(kFourWorldPos.At(0), kFourWorldPos.At(1), kFourWorldPos.At(2)));
            Node kFive = new Node(new Vector3(kFiveMatPos.At(0), kFiveMatPos.At(1), kFiveMatPos.At(2)), new Vector3(kFiveWorldPos.At(0), kFiveWorldPos.At(1), kFiveWorldPos.At(2)));
            new_nodes.Add(kFour);
            new_nodes.Add(kFive);

            Tetrahedron t1;
            Tetrahedron t2;
            Tetrahedron t3;

            if (count_of_positive_nodes == 1)
            {
                t1 = new Tetrahedron(kZeroPlus, kOne, kFour, kFive);
                t2 = new Tetrahedron(kZeroMinus, kTwo, kThree, kFour);
                t3 = new Tetrahedron(kZeroMinus, kTwo, kFour, kFive);

                if (t2.Intersect(t3))
                {
                    t3 = new Tetrahedron(kZeroMinus, kThree, kFour, kFive);
                }
            }
            else
            {
                t1 = new Tetrahedron(kZeroMinus, kOne, kFour, kFive);
                t2 = new Tetrahedron(kZeroPlus, kTwo, kThree, kFour);
                t3 = new Tetrahedron(kZeroPlus, kTwo, kFour, kFive);

                if (t2.Intersect(t3))
                {
                    t3 = new Tetrahedron(kZeroPlus, kThree, kFour, kFive);
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

                if(neighbor.Value.Contains(first_intersected_edge_n1) && neighbor.Value.Contains(first_intersected_edge_n2))
                {
                    first_connected_edge_intersected = true;
                }
                if(neighbor.Value.Contains(second_intersected_edge_n1) && neighbor.Value.Contains(second_intersected_edge_n2))
                {
                    second_connected_edge_intersected = true;
                }

                if(first_connected_edge_intersected || second_connected_edge_intersected)
                {
                    old_tetrahedra.Add(neighbor.Key);
                }

                if(first_connected_edge_intersected ^ second_connected_edge_intersected)
                {
                    Node w = null, x = null, y = null, z = null; // x & y are the nodes attached to the intersected edges, w & z are the other ones.
                    Node a = MathUtility.WhichNodeIsOneLine(n1, n2, kFour, kFive); // new node at the intersection point

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

                    // detect n3 and n4
                    bool first_found = false;
                    foreach (Node n in neighbor.Key.GetNodes())
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

                    Tetrahedron n_t1 = new Tetrahedron(a, x, w, z);
                    Tetrahedron n_t2 = new Tetrahedron(a, y, w, z);
                    new_tetrahedra.Add(n_t1);
                    new_tetrahedra.Add(n_t2);
                }

                if(first_connected_edge_intersected && second_connected_edge_intersected)
                {
                    Node x = null; // this is the node which is shared by both intersected edges
                    Node a = null, b = null; // these are the nodes which are at the intersection points
                    Node z = null; // this is the only node which is not attached to any of the intersected edges
                    Node w = null, y = null; // these are the other nodes which define the endpoint of the intersected edges but are not x.

                    if(first_intersected_edge_n1 == second_intersected_edge_n1)
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

                    a = MathUtility.WhichNodeIsOneLine(x, w, kFour, kFive);
                    b = MathUtility.WhichNodeIsOneLine(x, y, kFour, kFive);

                    foreach(Node n in neighbor.Key.GetNodes())
                    {
                        if(!n.Equals(x) && !n.Equals(y) && !n.Equals(w))
                        {
                            z = n;
                        }
                    }

                    Tetrahedron n_t1 = new Tetrahedron(a, b, x, z);
                    Tetrahedron n_t2 = new Tetrahedron(z, w, y, a);
                    Tetrahedron n_t3 = new Tetrahedron(z, w, a, b);
                    if (n_t2.Intersect(n_t2))
                    {
                        n_t3 = new Tetrahedron(z, y, a, b);
                    }

                    new_tetrahedra.Add(n_t1);
                    new_tetrahedra.Add(n_t2);
                    new_tetrahedra.Add(n_t3);
                }
            }
        }
        // TODO remove kZero from list of all nodes
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

        Matrix<float> st = (1/2.0f) * (-MathUtility.M(sumOverTensileForces) + sumOfMOfTensileForces + MathUtility.M(sumOverCompressiveForces) - sumOfMOfCompressiveForces);

        Evd<float> evd = st.Evd(Symmetricity.Unknown);
        float max = Mathf.Max(Mathf.Max((float) evd.EigenValues.At(0).Real,(float) evd.EigenValues.At(1).Real),(float) evd.EigenValues.At(2).Real);

        // TODO calculate corresponding eigenvector
        // assume it has the same index as the max eigenvalue

        if (toughness < max)
        {
            if(max == evd.EigenValues.At(0).Real)
            {
                fracture_plane_normal = evd.EigenVectors.Column(0);
            }
            else if (max == evd.EigenValues.At(1).Real)
            {
                fracture_plane_normal = evd.EigenVectors.Column(1);
            }
            else if (max == evd.EigenValues.At(2).Real)
            {
                fracture_plane_normal = evd.EigenVectors.Column(2);
            }
            return true; 
        }

        return false;
    }
}

public class Cuboid
{ 
    public Vector3[] point_positions = new Vector3[8];
    public Node[] nodes = new Node[8];

    public Cuboid(Vector3[] positions)
    {
        for(int i = 0; i < positions.Length; i++)
        {
            point_positions[i] = positions[i];
        }
    }

    public List<Node> MakeNodesFromPoints(Vector3 world_position)
    {
        List<Node> nodes = new List<Node>();

        foreach(Vector3 p in point_positions)
        {
            nodes.Add(new Node(p, p + world_position));
        }

        return nodes;
    }

    public List<Cuboid> Subdivide()
    {
        List<Cuboid> newCuboids = new List<Cuboid>();

        Vector3[] connections = FindConnectedPoints(0);

        Vector3 ConX = (connections[0] - point_positions[0]) / 2.0f;
        Vector3 ConY = (connections[1] - point_positions[0]) / 2.0f;
        Vector3 ConZ = (connections[2] - point_positions[0]) / 2.0f;

        Cuboid referenceCuboid = CreateCuboid(point_positions[0], ConX, ConY, ConZ);
        newCuboids.Add(referenceCuboid);
        
        for(int i = 1; i < 8; i++)
        {
            newCuboids.Add(CreateCuboid(referenceCuboid.point_positions[i], ConX, ConY, ConZ));
        }

        return newCuboids;
    }

    public Vector3[] FindConnectedPoints(int index)
    {
        Vector3[] connections = new Vector3[3];

        Vector3 start = point_positions[index];

        for(int i = 0; i < 8; i++)
        {
            Vector3 current = point_positions[i];
            if (i != index)
            {
                if (start.x == current.x && start.y == current.y)
                {
                    connections[2] = current;
                    Debug.Assert(i == 1);
                }
                else if (start.x == current.x && start.z == current.z)
                {
                    connections[1] = current;
                    Debug.Assert(i == 4);
                }
                else if (start.y == current.y && start.z == current.z)
                {
                    connections[0] = current;
                    Debug.Assert(i == 3);
                }
            }
        }

        return connections;
    }

    public Cuboid CreateCuboid(Vector3 location, Vector3 connectedToLocationOnX, Vector3 connectedToLocationOnY, Vector3 connectedToLocationOnZ)
    {
        Vector3[] vertices = new Vector3[8];

        vertices[0] = location;
        vertices[1] = location + connectedToLocationOnZ;
        vertices[2] = location + connectedToLocationOnX + connectedToLocationOnZ;
        vertices[3] = location + connectedToLocationOnX;
        vertices[4] = location + connectedToLocationOnY;
        vertices[5] = location + connectedToLocationOnY + connectedToLocationOnZ;
        vertices[6] = location + connectedToLocationOnX + connectedToLocationOnY + connectedToLocationOnZ;
        vertices[7] = location + connectedToLocationOnX + connectedToLocationOnY;

        return new Cuboid(vertices);
    }

    public List<Tetrahedron> GenerateTetrahedra()
    {
        List<Tetrahedron> tetrahedra = new List<Tetrahedron>();

        tetrahedra.Add(new Tetrahedron(nodes[2], nodes[5], nodes[6], nodes[7]));
        tetrahedra.Add(new Tetrahedron(nodes[0], nodes[4], nodes[5], nodes[7]));
        tetrahedra.Add(new Tetrahedron(nodes[0], nodes[2], nodes[3], nodes[7]));
        tetrahedra.Add(new Tetrahedron(nodes[0], nodes[1], nodes[2], nodes[5]));
        tetrahedra.Add(new Tetrahedron(nodes[0], nodes[2], nodes[5], nodes[7]));

        return tetrahedra;
    }
}

public class Tetrahedron
{
    Node[] nodes = new Node[4];
    Matrix<float> beta;
    public Dictionary<Tetrahedron, List<Node>> neighbors = new Dictionary<Tetrahedron, List<Node>>();

    public Tetrahedron(Node n1, Node n2, Node n3, Node n4)
    {
        nodes[0] = n1;
        nodes[1] = n2;
        nodes[2] = n3;
        nodes[3] = n4;
        beta = Beta();
    }

    public List<Tuple<Node, Node, Vector<float>>> Intersect(Vector<float> world_pos, Vector<float> normal_of_plane)
    {
        List<Tuple<Node, Node, Vector<float>>> intersection_points = new List<Tuple<Node, Node, Vector<float>>>();

        // TODO

        return intersection_points;
    }

    public bool Intersect(Tetrahedron other)
    {
        bool does_intersect = false;

        // TODO

        return does_intersect;
    }

    public Node[] GetNodes()
    {
        return nodes;
    }

    public void SwapNodes(Node tbs, Node new_node)
    {
        int index_of_tbs = -1;
        for(int i = 0; i < 4; i++)
        {
            if(nodes[i].Equals(tbs))
            {
                index_of_tbs = i;
            }
        }

        Debug.Assert(index_of_tbs != -1);
        nodes[index_of_tbs] = new_node;
    }

    public void DrawEdges()
    {
        Debug.DrawLine(nodes[0].world_pos, nodes[1].world_pos);
        Debug.DrawLine(nodes[0].world_pos, nodes[2].world_pos);
        Debug.DrawLine(nodes[0].world_pos, nodes[3].world_pos);
        Debug.DrawLine(nodes[1].world_pos, nodes[2].world_pos);
        Debug.DrawLine(nodes[1].world_pos, nodes[3].world_pos);
        Debug.DrawLine(nodes[2].world_pos, nodes[3].world_pos);
    }

    public void AddNeighbor(Tetrahedron neighbor, Node node)
    {
        if(!neighbors.ContainsKey(neighbor))
        {
            neighbors[neighbor] = new List<Node>();
        }
        neighbors[neighbor].Add(node);
    }

    public float Volume()
    {
        return 1 / 6.0f * Mathf.Abs(Vector3.Dot(Vector3.Cross(nodes[1].mat_pos - nodes[0].mat_pos, nodes[2].mat_pos - nodes[0].mat_pos), (nodes[3].mat_pos - nodes[0].mat_pos)));
    }

    public Matrix<float> ElemLoc()
    {
        return Matrix<float>.Build.DenseOfRowArrays(new float[][]
        {
            new float[] { nodes[0].world_pos.x, nodes[1].world_pos.x, nodes[2].world_pos.x, nodes[3].world_pos.x}, 
            new float[] { nodes[0].world_pos.y, nodes[1].world_pos.y, nodes[2].world_pos.y, nodes[3].world_pos.y},
            new float[] { nodes[0].world_pos.z, nodes[1].world_pos.z, nodes[2].world_pos.z, nodes[3].world_pos.z},
        });
    }

    public Matrix<float> ElemSpeed()
    {
        return Matrix<float>.Build.DenseOfRowArrays(new float[][]
        {
            new float[] { nodes[0].world_speed.x, nodes[1].world_speed.x, nodes[2].world_speed.x, nodes[3].world_speed.x},
            new float[] { nodes[0].world_speed.y, nodes[1].world_speed.y, nodes[2].world_speed.y, nodes[3].world_speed.y},
            new float[] { nodes[0].world_speed.z, nodes[1].world_speed.z, nodes[2].world_speed.z, nodes[3].world_speed.z},
        });
    }

    public Matrix<float> Beta()
    {
        Matrix<float> beta = Matrix<float>.Build.DenseOfRowArrays(new float[][]
        {
            new float[] { nodes[0].mat_pos.x, nodes[1].mat_pos.x, nodes[2].mat_pos.x, nodes[3].mat_pos.x},
            new float[] { nodes[0].mat_pos.y, nodes[1].mat_pos.y, nodes[2].mat_pos.y, nodes[3].mat_pos.y},
            new float[] { nodes[0].mat_pos.z, nodes[1].mat_pos.z, nodes[2].mat_pos.z, nodes[3].mat_pos.z},
            new float[] { 1.0f, 1.0f, 1.0f, 1.0f},
        });

        return beta.Inverse();
    }

    public Vector<float> LocInterp(Matrix<float> element_location, int i)
    {
        Vector<float> kron_delta_vector = MathUtility.KroneckerDeltaVector(i + 1);
        Debug.Assert(kron_delta_vector.Count == 4);

        return element_location * beta * kron_delta_vector;
    }

    public Vector<float> SpeedInterp(Matrix<float> element_speed, int i)
    {
        Vector<float> kron_delta_vector = MathUtility.KroneckerDeltaVector(i + 1);
        Debug.Assert(kron_delta_vector.Count == 4);

        return element_speed * beta * kron_delta_vector;
    }

    /* This is a strain metric, that only measures deformation and is invariant with respect
    to rigidbody transformations. */
    public Matrix<float> GreenStrain(Vector<float> li0, Vector<float> li1, Vector<float> li2)
    {
        Matrix<float> gs = Matrix<float>.Build.DenseOfRowArrays(new float[][]
        {
            new float[] { li0 * li0 - 1.0f, li1 * li0, li2 * li0},
            new float[] { li0 * li1, li1 * li1 - 1.0f, li2 * li1},
            new float[] { li0 * li2, li1 * li2, li2 * li2 - 1.0f}
        });

        Debug.Assert(gs.Column(0).Count == 3);
        Debug.Assert(gs.Row(0).Count == 3);

        return gs;
    }

    /* This is a strain rate metric, that measures how the strain is changing over time. Though if the strain is zero, 
    the strain reate is zero too. */
    public Matrix<float> StrainRate(Matrix<float> element_speed, Vector<float> li0, Vector<float> li1, Vector<float> li2)
    {
        Vector<float> si0 = SpeedInterp(element_speed, 0);
        Vector<float> si1 = SpeedInterp(element_speed, 1);
        Vector<float> si2 = SpeedInterp(element_speed, 2);

        Debug.Assert(si0.Count == 3);
        Debug.Assert(si1.Count == 3);
        Debug.Assert(si2.Count == 3);

        Matrix<float> sr = Matrix<float>.Build.DenseOfRowArrays(new float[][]
        {
            new float[] { (li0 * si0) + (si0 * li0), (li0 * si1) + (si0 * li1), (li0 * si2) + (si0 * li2) },
            new float[] { (li1 * si0) + (si1 * li0), (li1 * si1) + (si1 * li1), (li1 * si2) + (si1 * li2) },
            new float[] { (li2 * si0) + (si2 * li0), (li2 * si1) + (si2 * li1), (li2 * si2) + (si2 * li2) }
        });

        Debug.Assert(sr.Column(0).Count == 3);
        Debug.Assert(sr.Row(0).Count == 3);

        return sr;
    }


    /* This is an elastic stress metric, takes the green strain and properties of the material being modeled,
    and produces a matrix which holds information of the internal elastic stress due to the strain. */
    public Matrix<float> ElasticStressDueToStrain(float dilation, float rigidity, Vector<float> li0, Vector<float> li1, Vector<float> li2)
    {
        Matrix<float> gs = GreenStrain(li0, li1, li2);
        Matrix<float> esdts = Matrix<float>.Build.DenseOfRowArrays(new float[][]
        {
            new float[] { 0.0f, 0.0f, 0.0f },
            new float[] { 0.0f, 0.0f, 0.0f },
            new float[] { 0.0f, 0.0f, 0.0f }
        });

        for (int row = 0; row < 3; row++)
        {
            for(int column = 0; column < 3; column++)
            {
                float tmp = 0.0f;
                for (int k = 0; k < 3; k++)
                {
                    tmp += dilation * gs.At(k, k) * MathUtility.KroneckerDelta(row, column);
                }
                tmp += 2.0f * rigidity * gs.At(row, column);
                esdts.At(row, column, tmp);
            }
        }

        return esdts;
    }

    public Matrix<float> ViscousStressDueToStrainRate(float phi, float psi, Matrix<float> element_speed, Vector<float> li0, Vector<float> li1, Vector<float> li2)
    {
        Matrix<float> sr = StrainRate(element_speed, li0, li1, li2);
        Matrix<float> vsdtsr = Matrix<float>.Build.DenseOfRowArrays(new float[][]
        {
            new float[] { 0.0f, 0.0f, 0.0f },
            new float[] { 0.0f, 0.0f, 0.0f },
            new float[] { 0.0f, 0.0f, 0.0f }
        });

        for (int row = 0; row < 3; row++)
        {
            for (int column = 0; column < 3; column++)
            {
                float tmp = 0.0f;
                for (int k = 0; k < 3; k++)
                {
                    tmp += phi * sr.At(k, k) * MathUtility.KroneckerDelta(row, column);
                }
                tmp += 2.0f * psi * sr.At(row, column);
                vsdtsr.At(row, column, tmp);
            }
        }

        return vsdtsr;
    }

    public Matrix<float> TotalInternalStress(float dilation, float rigidity, float phi, float psi)
    {
        Matrix<float> element_location = ElemLoc();
        Debug.Assert(element_location.Row(0).Count == 4);
        Debug.Assert(element_location.Column(0).Count == 3);

        Vector<float> li0 = LocInterp(element_location, 0);
        Vector<float> li1 = LocInterp(element_location, 1);
        Vector<float> li2 = LocInterp(element_location, 2);

        Matrix<float> element_speed = ElemSpeed();
        Debug.Assert(element_speed.Row(0).Count == 4);
        Debug.Assert(element_speed.Column(0).Count == 3);

        return ElasticStressDueToStrain(dilation, rigidity, li0, li1, li2) + ViscousStressDueToStrainRate(phi, psi, element_speed, li0, li1, li2);
    }

    public Matrix<float> TensileComponentOfTotalInternalStress(Evd<float> evd_of_total_internal_stress)
    {
        Matrix<float> tcotis = Matrix<float>.Build.DenseOfRowArrays(new float [][]
        { 
            new float[] { 0.0f, 0.0f, 0.0f },
            new float[] { 0.0f, 0.0f, 0.0f },
            new float[] { 0.0f, 0.0f, 0.0f }
        });
        for(int i = 0; i < 3; i++)
        {
            Vector<float> nev = evd_of_total_internal_stress.EigenVectors.Column(i);
            float nev_len = (float) nev.L2Norm();
            nev.At(0, nev.At(0) / nev_len);
            nev.At(1, nev.At(1) / nev_len);
            nev.At(2, nev.At(2) / nev_len);

            tcotis += Mathf.Max(0.0f, (float) evd_of_total_internal_stress.EigenValues.At(i).Real) * MathUtility.M(nev);
        }

        return tcotis;
    }

    public Matrix<float> CompressiveComponentOfTotalInternalStress(Evd<float> evd_of_total_internal_stress)
    {
        Matrix<float> ccotis = Matrix<float>.Build.DenseOfRowArrays(new float[][]
        {
            new float[] { 0.0f, 0.0f, 0.0f },
            new float[] { 0.0f, 0.0f, 0.0f },
            new float[] { 0.0f, 0.0f, 0.0f }
        });
        for (int i = 0; i < 3; i++)
        {
            Vector<float> nev = evd_of_total_internal_stress.EigenVectors.Column(i);
            float nev_len = (float) nev.L2Norm();
            nev.At(0, nev.At(0) / nev_len);
            nev.At(1, nev.At(1) / nev_len);
            nev.At(2, nev.At(2) / nev_len);

            ccotis += Mathf.Min(0.0f, (float) evd_of_total_internal_stress.EigenValues.At(i).Real) * MathUtility.M(nev);
        }

        return ccotis;
    }

    public Vector<float> TensileForce(float volume, Evd<float> evd_of_total_internal_stress, int node_index)
    {
        Matrix<float> tcotis = TensileComponentOfTotalInternalStress(evd_of_total_internal_stress);
        Vector<float> tf = Vector<float>.Build.DenseOfArray(new float[] { 0.0f, 0.0f, 0.0f});

        for(int nodePosition = 0; nodePosition < 4; nodePosition++)
        {
            float scalar = 0.0f;
            for(int k = 0; k < 3; k++)
            {
                for(int l = 0; l < 3; l++)
                {
                    scalar += beta.At(nodePosition, l) * beta.At(node_index, k) * tcotis.At(k, l);
                }
            }
            Vector<float> node_world_pos = Vector<float>.Build.DenseOfArray(new float[] 
            { 
                nodes[nodePosition].world_pos.x, 
                nodes[nodePosition].world_pos.y, 
                nodes[nodePosition].world_pos.z 
            });

            tf += node_world_pos * scalar;
        }

        tf *= (-volume / 2.0f);

        return tf;
    }

    public Vector<float> CompressiveForce(float volume, Evd<float> evd_of_total_internal_stress, int node_index)
    {
        Matrix<float> ccotis = CompressiveComponentOfTotalInternalStress(evd_of_total_internal_stress);
        Vector<float> tf = Vector<float>.Build.DenseOfArray(new float[] { 0.0f, 0.0f, 0.0f });

        for (int nodePosition = 0; nodePosition < 4; nodePosition++)
        {
            float scalar = 0.0f;
            for (int k = 0; k < 3; k++)
            {
                for (int l = 0; l < 3; l++)
                {
                    scalar += beta.At(nodePosition, l) * beta.At(node_index, k) * ccotis.At(k, l);
                }
            }
            Vector<float> node_world_pos = Vector<float>.Build.DenseOfArray(new float[]
            {
                nodes[nodePosition].world_pos.x,
                nodes[nodePosition].world_pos.y,
                nodes[nodePosition].world_pos.z
            });

            tf += node_world_pos * scalar;
        }

        tf *= (-volume / 2.0f);

        return tf;
    }

    public void UpdateInternalForcesOfNodes(float dilation, float rigidity, float phi, float psi)
    {
        Matrix<float> total_internal_stress = TotalInternalStress(dilation, rigidity, phi, psi);
        Evd<float> evd_of_total_internal_stress = total_internal_stress.Evd(Symmetricity.Symmetric);
        float volume = Volume();

        for (int i = 0; i < 4; i++)
        {
            nodes[i].AddTensileForce(TensileForce(volume, evd_of_total_internal_stress, i));
            nodes[i].AddCompressiveForce(CompressiveForce(volume, evd_of_total_internal_stress, i));
        }
    }
}

public class FractureSimulator : MonoBehaviour
{
    [SerializeField]
    Transform sphere;

    [SerializeField]
    int Subdivisions;
    
    Mesh mesh;
    Vector3[] vertices = new Vector3[8];

    List<Cuboid> allcuboids = new List<Cuboid>();
    List<Tetrahedron> alltetrahedra = new List<Tetrahedron>();
    List<Node> allnodes = new List<Node>();

    public const float GLASS_DILATION = 1.04e8f;
    public const float GLASS_RIGIDITY = 1.04e8f;
    public const float GLASS_PSI = 0.0f;
    public const float GLASS_PHI = 6760.0f;
    public const float GLASS_DENSITY = 2588.0f;
    public const float GLASS_TOUGHNESS = 10140.0f;

    void Update()
    {
        Vector3 x = new Vector3(0.01f * Time.deltaTime, -0.01f * Time.deltaTime, 0.01f * Time.deltaTime);
        allnodes[0].new_world_pos += x;
        foreach (Node n in allnodes)
        {
            // n.new_world_pos += new Vector3(1f * Time.deltaTime, 0.0f, 0.0f);
            n.world_speed = (n.new_world_pos - n.world_pos) / Time.deltaTime;
            n.world_pos = n.new_world_pos;
        }

        foreach (Tetrahedron t in alltetrahedra)
        {
            t.UpdateInternalForcesOfNodes(GLASS_DILATION, GLASS_RIGIDITY, GLASS_PHI, GLASS_PSI);
        }

        foreach (Node n in allnodes)
        {
            if (n.DoesCrackOccur(GLASS_TOUGHNESS))
            {
                Debug.Log("Crack occured at Node " + n.mat_pos.x + " " + n.mat_pos.y + " " + n.mat_pos.z);
                Instantiate(sphere, n.world_pos, Quaternion.identity);

                n.Crack(transform.position);
            }
            n.ClearTensileAndCompressiveForces();
        }

        foreach (Tetrahedron t in alltetrahedra)
        {
            t.DrawEdges();
        }
    }

    public void FixedUpdate()
    {
    }

    void Start()
    {
        Time.fixedDeltaTime = 0.1f;

        // retrieve local vertex positions
        mesh = gameObject.GetComponent<MeshFilter>().mesh;
        Vector3[] temp = mesh.vertices.Distinct().ToArray();
        vertices[0] = temp[5];
        vertices[1] = temp[3];
        vertices[2] = temp[2];
        vertices[3] = temp[4];
        vertices[4] = temp[7];
        vertices[5] = temp[1];
        vertices[6] = temp[0];
        vertices[7] = temp[6];

        Cuboid main_cuboid = new Cuboid(vertices);
        allcuboids.Add(main_cuboid);

        // subdivide the cuboid into more cuboids
        for (int subdiv_index = 0; subdiv_index < Subdivisions; subdiv_index++)
        {
            List<Cuboid> working_copy = new List<Cuboid>();
            foreach (Cuboid c in allcuboids)
            {
                working_copy.Add(c);
            }
            allcuboids.Clear();
            for (int cuboid_index = 0; cuboid_index < working_copy.Count; cuboid_index++)
            {
                Cuboid current_cuboid = working_copy[cuboid_index];
                allcuboids.AddRange(current_cuboid.Subdivide());
            }
        }

        // create nodes
        for (int cuboid = 0; cuboid < allcuboids.Count; cuboid++)
        {
            allnodes.AddRange(allcuboids[cuboid].MakeNodesFromPoints(gameObject.transform.position));
        }

        // clear duplicates
        for (int node_1 = 0; node_1 < allnodes.Count; node_1++)
        {
            int rerun_counter = 0;
            for (int node_2 = 0; node_2 < allnodes.Count; node_2++)
            {
                if (node_1 == node_2) continue;

                if (allnodes[node_1].world_pos == allnodes[node_2].world_pos)
                {
                    allnodes.Remove(allnodes[node_2]);
                    node_2--;
                    rerun_counter++;
                }
            }
            node_1 -= rerun_counter;
        }

        // find nodes for cuboids
        for (int cuboid = 0; cuboid < allcuboids.Count; cuboid++)
        {
            for (int corner_pos = 0; corner_pos < 8; corner_pos++)
            {
                for (int node = 0; node < allnodes.Count; node++)
                {
                    if (allcuboids[cuboid].point_positions[corner_pos] == allnodes[node].mat_pos)
                    {
                        allcuboids[cuboid].nodes[corner_pos] = allnodes[node];
                    }
                }
            }
        }

        // create tetrahedra
        for (int cuboid = 0; cuboid < allcuboids.Count; cuboid++)
        {
            alltetrahedra.AddRange(allcuboids[cuboid].GenerateTetrahedra());
        }

        // for each node, find all attached tetrahedra
        foreach(Tetrahedron t in alltetrahedra)
        {
            foreach(Node n in t.GetNodes())
            {
                n.attached_elements.Add(t);
            }
        }

        // find neighbors
        Node[] nodes_t1;
        Node[] nodes_t2;
        for (int t1 = 0; t1 < alltetrahedra.Count; t1++)
        {
            for(int t2 = 0; t2 < alltetrahedra.Count; t2++)
            {
                if (t1 == t2) continue;
                nodes_t1 = alltetrahedra[t1].GetNodes();
                nodes_t2 = alltetrahedra[t2].GetNodes();
                foreach (Node n1 in nodes_t1)
                {
                    foreach(Node n2 in nodes_t2)
                    {
                        if(n1.Equals(n2))
                        {
                            alltetrahedra[t1].AddNeighbor(alltetrahedra[t2], n1);
                            alltetrahedra[t2].AddNeighbor(alltetrahedra[t1], n1);
                        }
                    }
                }
            }
        }

        Debug.Log("There are " + allnodes.Count + " Nodes and " + alltetrahedra.Count + " Tetrahedra.");
    }
}
