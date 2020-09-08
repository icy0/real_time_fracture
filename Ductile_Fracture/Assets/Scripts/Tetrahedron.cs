using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System;
using UnityEngine;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Factorization;

public class Tetrahedron : MonoBehaviour
{
    public Node[] nodes = new Node[4];
    public List<Transform> node_transforms = new List<Transform>();
    public Dictionary<Tetrahedron, List<Node>> neighbors = new Dictionary<Tetrahedron, List<Node>>();
    public int neighbors_count;

    public float volume;
    private Matrix<float> beta;

    public void AddNode(Transform n)
    {
        node_transforms.Add(n);
    }

    public void Start()
    {
        Beta();
    }

    public void Beta()
    {
        beta = Matrix<float>.Build.DenseOfRowArrays(new float[][]
        {
            new float[] { nodes[0].transform.localPosition.x, nodes[1].transform.localPosition.x, nodes[2].transform.localPosition.x, nodes[3].transform.localPosition.x},
            new float[] { nodes[0].transform.localPosition.y, nodes[1].transform.localPosition.y, nodes[2].transform.localPosition.y, nodes[3].transform.localPosition.y},
            new float[] { nodes[0].transform.localPosition.z, nodes[1].transform.localPosition.z, nodes[2].transform.localPosition.z, nodes[3].transform.localPosition.z},
            new float[] { 1.0f, 1.0f, 1.0f, 1.0f},
        });

        beta = beta.Inverse();
    }

    public Tuple<bool /*is there an edge of the tet that is parallel to the fracture plane and is the fp NOT parallel to a face*/, 
        bool /*is the fracture plane coplanar with a face?*/, 
        List<Tuple<Node, Node, Vector<float>>>> 
        Intersect(Node n, Vector<float> normal_of_plane)
    {
        List<Tuple<Node, Node, Vector<float>>> intersection_points = new List<Tuple<Node, Node, Vector<float>>>();
        Vector<float> world_pos_of_n = Vector<float>.Build.DenseOfArray(new float[] { n.transform.position.x, n.transform.position.y, n.transform.position.z });
        bool coplanar_with_face = false;
        bool parallel_to_edge_but_not_coplanar = false;

        // only do intersection for edges between nodes that are not n
        Node x, y, z;
        int i = MathUtility.GetIndexOf(this, n);

        x = this.nodes[(i + 1) % 4];
        y = this.nodes[(i + 2) % 4];
        z = this.nodes[(i + 3) % 4];

        Vector<float> world_pos_of_x = Vector<float>.Build.DenseOfArray(new float[] { x.transform.position.x, x.transform.position.y, x.transform.position.z });
        Vector<float> world_pos_of_y = Vector<float>.Build.DenseOfArray(new float[] { y.transform.position.x, y.transform.position.y, y.transform.position.z });
        Vector<float> world_pos_of_z = Vector<float>.Build.DenseOfArray(new float[] { z.transform.position.x, z.transform.position.y, z.transform.position.z });

        Tuple<bool, bool, Vector<float>> intersection_xy = MathUtility.LinePlaneIntersection(world_pos_of_n, normal_of_plane, world_pos_of_x, world_pos_of_y);
        Tuple<bool, bool, Vector<float>> intersection_xz = MathUtility.LinePlaneIntersection(world_pos_of_n, normal_of_plane, world_pos_of_x, world_pos_of_z);
        Tuple<bool, bool, Vector<float>> intersection_yz = MathUtility.LinePlaneIntersection(world_pos_of_n, normal_of_plane, world_pos_of_y, world_pos_of_z);

        // TODO if the intersection point is too close to a node, just set it to be on the node and update the booleans. this way we avoid marginally small tetrahedra.

        if (intersection_xy.Item1) intersection_points.Add(new Tuple<Node, Node, Vector<float>>(x, y, intersection_xy.Item3));
        if (intersection_xz.Item1) intersection_points.Add(new Tuple<Node, Node, Vector<float>>(x, z, intersection_xz.Item3));
        if (intersection_yz.Item1) intersection_points.Add(new Tuple<Node, Node, Vector<float>>(y, z, intersection_yz.Item3));

        // if any of the edges between x, y and z is parallel to the fracture plane, then the fp is coplanar to a face
        if (intersection_xy.Item2) coplanar_with_face = true;
        if (intersection_xz.Item2) coplanar_with_face = true;
        if (intersection_yz.Item2) coplanar_with_face = true;

        // it could still be that one edge coming from n is parallel to the fracture plane
        if (!coplanar_with_face)
        {
            // see if intersection is exactly at x, y, or z
            bool is_inters_at_x = intersection_xy.Item1 && intersection_xz.Item1 && (MathUtility.EqualsRoughly(world_pos_of_x, intersection_xy.Item3, 0.0f) || MathUtility.EqualsRoughly(world_pos_of_x, intersection_xz.Item3, 0.0f));
            bool is_inters_at_y = intersection_xy.Item1 && intersection_yz.Item1 && (MathUtility.EqualsRoughly(world_pos_of_y, intersection_xy.Item3, 0.0f) || MathUtility.EqualsRoughly(world_pos_of_y, intersection_yz.Item3, 0.0f));
            bool is_inters_at_z = intersection_xz.Item1 && intersection_yz.Item1 && (MathUtility.EqualsRoughly(world_pos_of_z, intersection_xz.Item3, 0.0f) || MathUtility.EqualsRoughly(world_pos_of_z, intersection_yz.Item3, 0.0f));

            if (is_inters_at_x || is_inters_at_y || is_inters_at_z)
            {
                parallel_to_edge_but_not_coplanar = true;
            }
        }

        return new Tuple<bool, bool, List<Tuple<Node, Node, Vector<float>>>>(parallel_to_edge_but_not_coplanar, coplanar_with_face, intersection_points);
    }

    public void SwapNodes(Node tbs, Node new_node)
    {
        int index_of_tbs = MathUtility.GetIndexOf(this, tbs);
        Debug.Assert(index_of_tbs != -1);

        nodes[index_of_tbs] = new_node;
        node_transforms[index_of_tbs] = new_node.transform;
    }

    public void DrawEdges()
    {
        Debug.DrawLine(nodes[0].transform.position, nodes[1].transform.position);
        Debug.DrawLine(nodes[0].transform.position, nodes[2].transform.position);
        Debug.DrawLine(nodes[0].transform.position, nodes[3].transform.position);
        Debug.DrawLine(nodes[1].transform.position, nodes[2].transform.position);
        Debug.DrawLine(nodes[1].transform.position, nodes[3].transform.position);
        Debug.DrawLine(nodes[2].transform.position, nodes[3].transform.position);
    }

    public void AddNeighbor(Tetrahedron neighbor, Node node)
    {
        if (!neighbors.ContainsKey(neighbor))
        {
            neighbors[neighbor] = new List<Node>();
        }
        neighbors[neighbor].Add(node);
    }

    public float Volume()
    {
        return 1 / 6.0f * Mathf.Abs(Vector3.Dot(Vector3.Cross(nodes[1].transform.localPosition - nodes[0].transform.localPosition, nodes[2].transform.localPosition - nodes[0].transform.localPosition), (nodes[3].transform.localPosition - nodes[0].transform.localPosition)));
    }

    public Matrix<float> ElemLoc()
    {
        return Matrix<float>.Build.DenseOfRowArrays(new float[][]
        {
            new float[] { nodes[0].transform.position.x, nodes[1].transform.position.x, nodes[2].transform.position.x, nodes[3].transform.position.x},
            new float[] { nodes[0].transform.position.y, nodes[1].transform.position.y, nodes[2].transform.position.y, nodes[3].transform.position.y},
            new float[] { nodes[0].transform.position.z, nodes[1].transform.position.z, nodes[2].transform.position.z, nodes[3].transform.position.z},
        });
    }

    public Matrix<float> ElemSpeed()
    {
        return Matrix<float>.Build.DenseOfRowArrays(new float[][]
        {
            new float[] { nodes[0].velocity.x, nodes[1].velocity.x, nodes[2].velocity.x, nodes[3].velocity.x},
            new float[] { nodes[0].velocity.y, nodes[1].velocity.y, nodes[2].velocity.y, nodes[3].velocity.y},
            new float[] { nodes[0].velocity.z, nodes[1].velocity.z, nodes[2].velocity.z, nodes[3].velocity.z},
        });
    }

    public Vector<float> LocInterp(Matrix<float> element_location, int i)
    {
        Vector<float> kron_delta_vector = MathUtility.KroneckerDeltaVector(i + 1);
        Debug.Assert(kron_delta_vector.Count == 4);
        Debug.Assert(element_location != null);
        Debug.Assert(beta != null, "Beta was null. Have a look at " + this.name);
        Debug.Assert(kron_delta_vector != null);

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

    public Matrix<float> ElasticStrain()
    {

    }

    /* This is an elastic stress metric, takes the green strain and properties of the material being modeled,
    and produces a matrix which holds information of the internal elastic stress due to the strain. */
    public Matrix<float> ElasticStressDueToStrain(float dilation, float rigidity, Vector<float> li0, Vector<float> li1, Vector<float> li2)
    {
        Matrix<float> gs = GreenStrain(li0, li1, li2); // TODO swap this with elastic strain
        Matrix<float> elastic_strain = ElasticStrain();
        Matrix<float> esdts = Matrix<float>.Build.DenseOfRowArrays(new float[][]
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

    // TODO assumption is made here: that Eigenvectors.Column(i) is corresponding to Eigenvalue.At(i). this is not necessarily correct.
    public Matrix<float> TensileComponentOfTotalInternalStress(Evd<float> evd_of_total_internal_stress)
    {
        Matrix<float> tcotis = Matrix<float>.Build.DenseOfRowArrays(new float[][]
        {
            new float[] { 0.0f, 0.0f, 0.0f },
            new float[] { 0.0f, 0.0f, 0.0f },
            new float[] { 0.0f, 0.0f, 0.0f }
        });
        for (int i = 0; i < 3; i++)
        {
            Vector<float> nev = evd_of_total_internal_stress.EigenVectors.Column(i);
            float nev_len = (float)nev.L2Norm();
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
            float nev_len = (float)nev.L2Norm();
            nev.At(0, nev.At(0) / nev_len);
            nev.At(1, nev.At(1) / nev_len);
            nev.At(2, nev.At(2) / nev_len);

            ccotis += Mathf.Min(0.0f, (float)evd_of_total_internal_stress.EigenValues.At(i).Real) * MathUtility.M(nev);
        }

        return ccotis;
    }

    public Vector<float> TensileForce(float volume, Evd<float> evd_of_total_internal_stress, int node_index)
    {
        Matrix<float> tcotis = TensileComponentOfTotalInternalStress(evd_of_total_internal_stress);
        Vector<float> tf = Vector<float>.Build.DenseOfArray(new float[] { 0.0f, 0.0f, 0.0f });

        for (int nodePosition = 0; nodePosition < 4; nodePosition++)
        {
            float scalar = 0.0f;
            for (int k = 0; k < 3; k++)
            {
                for (int l = 0; l < 3; l++)
                {
                    scalar += beta.At(nodePosition, l) * beta.At(node_index, k) * tcotis.At(k, l);
                }
            }
            Vector<float> node_world_pos = Vector<float>.Build.DenseOfArray(new float[]
            {
                nodes[nodePosition].transform.position.x,
                nodes[nodePosition].transform.position.y,
                nodes[nodePosition].transform.position.z
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
                nodes[nodePosition].transform.position.x,
                nodes[nodePosition].transform.position.y,
                nodes[nodePosition].transform.position.z
            });

            tf += node_world_pos * scalar;
        }

        tf *= (-volume / 2.0f);

        return tf;
    }

    public void UpdateInternalForcesOfNodes(float dilation, float rigidity, float phi, float psi)
    {
        Matrix<float> total_internal_stress = TotalInternalStress(dilation, rigidity, phi, psi);
        Debug.Assert(total_internal_stress.IsSymmetric(), "The total internal stress tensor is not symmetric.");

        Evd<float> evd_of_total_internal_stress = total_internal_stress.Evd(Symmetricity.Symmetric);
        volume = Volume();

        for (int i = 0; i < 4; i++)
        {
            nodes[i].AddTensileForce(TensileForce(volume, evd_of_total_internal_stress, i));
            nodes[i].AddCompressiveForce(CompressiveForce(volume, evd_of_total_internal_stress, i));
        }
    }
}
