using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using MathNet.Numerics;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using MathNet.Numerics.LinearAlgebra.Factorization;

public class MathUtility
{
    public static int KroneckerDelta(int i, int j)
    {
        if (i == j) return 1;
        else return 0;
    }

    public static Vector<double> KroneckerDeltaVector(int i)
    {
        Vector<double> kdv = Vector<double>.Build.DenseOfArray(new double[] { 0.0f, 0.0f, 0.0f, 0.0f });

        if (i == 1)
            kdv.At(0, 1.0f);
        else if (i == 2)
            kdv.At(1, 1.0f);
        else if (i == 3)
            kdv.At(2, 1.0f);

        return kdv;
    }

    public static Matrix<double> M(Vector<double> a)
    {
        if (!(a.At(0) == 0.0 && a.At(1) == 0.0 && a.At(2) == 0.0))
        {
            Matrix<double> m = Matrix<double>.Build.DenseOfRowArrays(new double[][]
            {
               new double[] { a.At(0) * a.At(0), a.At(0) * a.At(1), a.At(0) * a.At(2)},
               new double[] { a.At(1) * a.At(0), a.At(1) * a.At(1), a.At(1) * a.At(2)},
               new double[] { a.At(2) * a.At(0), a.At(2) * a.At(1), a.At(2) * a.At(2)}
            });
            return m / a.L2Norm();
        }
        else
        {
            return Matrix<double>.Build.DenseOfRowArrays(new double[][]
            {
               new double[] { 0.0, 0.0, 0.0},
               new double[] { 0.0, 0.0, 0.0},
               new double[] { 0.0, 0.0, 0.0}
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

    public HashSet<Vector<double>> tensile_forces = new HashSet<Vector<double>>();
    public HashSet<Vector<double>> compressive_forces = new HashSet<Vector<double>>();

    public Node(Vector3 m_pos, Vector3 w_pos)
    {
        mat_pos = m_pos;
        world_pos = w_pos;
        new_world_pos = w_pos;
        world_speed = new Vector3(0.0f, 0.0f, 0.0f);
    }

    public void AddTensileForce(Vector<double> tf)
    {
        tensile_forces.Add(tf);
    }

    public void AddCompressiveForce(Vector<double> cf)
    {
        compressive_forces.Add(cf);
    }

    public bool DoesCrackOccur(double toughness)
    {
        Vector<double> sumOverTensileForces = Vector<double>.Build.DenseOfArray(new double[] { 0.0, 0.0, 0.0 });
        Matrix<double> sumOfMOfTensileForces = Matrix<double>.Build.DenseOfRowArrays(new double[][]
        {
            new double[] { 0.0, 0.0, 0.0},
            new double[] { 0.0, 0.0, 0.0},
            new double[] { 0.0, 0.0, 0.0}
        });
        foreach (Vector<double> tf in tensile_forces)
        {
            sumOverTensileForces += tf;
            sumOfMOfTensileForces += MathUtility.M(tf);
        }

        Vector<double> sumOverCompressiveForces = Vector<double>.Build.DenseOfArray(new double[] { 0.0, 0.0, 0.0 });
        Matrix<double> sumOfMOfCompressiveForces = Matrix<double>.Build.DenseOfRowArrays(new double[][]
        {
            new double[] { 0.0, 0.0, 0.0},
            new double[] { 0.0, 0.0, 0.0},
            new double[] { 0.0, 0.0, 0.0}
        });
        foreach (Vector<double> cf in compressive_forces)
        {
            sumOverCompressiveForces += cf;
            sumOfMOfCompressiveForces += MathUtility.M(cf);
        }

        Matrix<double> st = (1/2.0) * (-MathUtility.M(sumOverTensileForces) + sumOfMOfTensileForces + MathUtility.M(sumOverCompressiveForces) - sumOfMOfCompressiveForces);

        Evd<double> evd = st.Evd(Symmetricity.Symmetric);
        if(toughness < evd.EigenValues.Maximum().Real)
        {
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

    // location is NOT the center of the cuboid, but one random point position
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

        // TODO bring into correct order
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
    public Tetrahedron(Node n1, Node n2, Node n3, Node n4)
    {
        nodes[0] = n1;
        nodes[1] = n2;
        nodes[2] = n3;
        nodes[3] = n4;
    }

    public Node[] GetNodes()
    {
        return nodes;
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

    public float Volume()
    {
        return 1 / 6.0f * Mathf.Abs(Vector3.Dot(Vector3.Cross(nodes[1].mat_pos - nodes[0].mat_pos, nodes[2].mat_pos - nodes[0].mat_pos), (nodes[3].mat_pos - nodes[0].mat_pos)));
    }

    public Matrix<double> ElemLoc()
    {
        return Matrix<double>.Build.DenseOfRowArrays(new double[][]
        {
            new double[] { nodes[0].world_pos.x, nodes[1].world_pos.x, nodes[2].world_pos.x, nodes[3].world_pos.x}, 
            new double[] { nodes[0].world_pos.y, nodes[1].world_pos.y, nodes[2].world_pos.y, nodes[3].world_pos.y},
            new double[] { nodes[0].world_pos.z, nodes[1].world_pos.z, nodes[2].world_pos.z, nodes[3].world_pos.z},
        });
    }

    public Matrix<double> ElemSpeed()
    {
        return Matrix<double>.Build.DenseOfRowArrays(new double[][]
        {
            new double[] { nodes[0].world_speed.x, nodes[1].world_speed.x, nodes[2].world_speed.x, nodes[3].world_speed.x},
            new double[] { nodes[0].world_speed.y, nodes[1].world_speed.y, nodes[2].world_speed.y, nodes[3].world_speed.y},
            new double[] { nodes[0].world_speed.z, nodes[1].world_speed.z, nodes[2].world_speed.z, nodes[3].world_speed.z},
        });
    }

    public Matrix<double> Beta()
    {
        Matrix<double> beta = Matrix<double>.Build.DenseOfRowArrays(new double[][]
        {
            new double[] { nodes[0].mat_pos.x, nodes[1].mat_pos.x, nodes[2].mat_pos.x, nodes[3].mat_pos.x},
            new double[] { nodes[0].mat_pos.y, nodes[1].mat_pos.y, nodes[2].mat_pos.y, nodes[3].mat_pos.y},
            new double[] { nodes[0].mat_pos.z, nodes[1].mat_pos.z, nodes[2].mat_pos.z, nodes[3].mat_pos.z},
            new double[] { 1.0f, 1.0f, 1.0f, 1.0f},
        });

        return beta.Inverse();
    }

    public Vector<double> LocInterp(int i)
    {
        return (ElemLoc() * Beta()) * MathUtility.KroneckerDeltaVector(i);
    }

    public Vector<double> SpeedInterp(int i)
    {
        return (ElemSpeed() * Beta()) * MathUtility.KroneckerDeltaVector(i);
    }

    public Matrix<double> GreenStrain()
    {
        return Matrix<double>.Build.DenseOfRowArrays(new double[][] 
        { 
            new double[] { LocInterp(0) * LocInterp(0) - 1.0f,  LocInterp(1) * LocInterp(0),  LocInterp(2) * LocInterp(0)},
            new double[] { LocInterp(0) * LocInterp(1),  LocInterp(1) * LocInterp(1) - 1.0f,  LocInterp(2) * LocInterp(1)},
            new double[] { LocInterp(0) * LocInterp(2),  LocInterp(1) * LocInterp(2),  LocInterp(2) * LocInterp(2) - 1.0f} 
        });
    }

    public Matrix<double> StrainRate()
    {
        return Matrix<double>.Build.DenseOfRowArrays(new double[][]
        {
            new double[] { (LocInterp(0) * SpeedInterp(0)) + (SpeedInterp(0) * LocInterp(0)), (LocInterp(0) * SpeedInterp(1)) + (SpeedInterp(0) * LocInterp(1)), (LocInterp(0) * SpeedInterp(2)) + (SpeedInterp(0) * LocInterp(2))},
            new double[] { (LocInterp(1) * SpeedInterp(0)) + (SpeedInterp(1) * LocInterp(0)), (LocInterp(1) * SpeedInterp(1)) + (SpeedInterp(1) * LocInterp(1)), (LocInterp(1) * SpeedInterp(2)) + (SpeedInterp(1) * LocInterp(2))},
            new double[] { (LocInterp(2) * SpeedInterp(0)) + (SpeedInterp(2) * LocInterp(0)), (LocInterp(2) * SpeedInterp(1)) + (SpeedInterp(2) * LocInterp(1)), (LocInterp(2) * SpeedInterp(2)) + (SpeedInterp(2) * LocInterp(2))}
        });
    }

    public Matrix<double> ElasticStressDueToStrain(double dilation, double rigidity)
    {
        Matrix<double> gs = GreenStrain();
        Matrix<double> esdts = Matrix<double>.Build.DenseOfRowArrays(new double[][]
        {
            new double[] { 0.0f, 0.0f, 0.0f },
            new double[] { 0.0f, 0.0f, 0.0f },
            new double[] { 0.0f, 0.0f, 0.0f }
        });

        for (int row = 0; row < 3; row++)
        {
            for(int column = 0; column < 3; column++)
            {
                double tmp = 0.0f;
                for (int i = 0; i < 3; i++)
                {
                    tmp += dilation * gs.At(i, i) * MathUtility.KroneckerDelta(row, column) + 2.0f * rigidity * gs.At(row, column);
                }
                esdts.At(row, column, tmp);
            }
        }

        return esdts;
    }

    public Matrix<double> ViscousStressDueToStrainRate(double phi, double psi)
    {
        Matrix<double> sr = StrainRate();
        Matrix<double> vsdtsr = Matrix<double>.Build.DenseOfRowArrays(new double[][]
        {
            new double[] { 0.0f, 0.0f, 0.0f },
            new double[] { 0.0f, 0.0f, 0.0f },
            new double[] { 0.0f, 0.0f, 0.0f }
        });

        for (int row = 0; row < 3; row++)
        {
            for (int column = 0; column < 3; column++)
            {
                double tmp = 0.0f;
                for (int i = 0; i < 3; i++)
                {
                    tmp += phi * sr.At(i, i) * MathUtility.KroneckerDelta(row, column) + 2.0f * psi * sr.At(row, column);
                }
                vsdtsr.At(row, column, tmp);
            }
        }

        return vsdtsr;
    }

    public Matrix<double> TotalInternalStress(double dilation, double rigidity, double phi, double psi)
    {
        return ElasticStressDueToStrain(dilation, rigidity) + ViscousStressDueToStrainRate(phi, psi);
    }

    public Matrix<double> TensileComponentOfTotalInternalStress(double dilation, double rigidity, double phi, double psi)
    {
        Evd<double> evd = TotalInternalStress(dilation, rigidity, phi, psi).Evd(Symmetricity.Symmetric);
        Matrix<double> tcotis = Matrix<double>.Build.DenseOfRowArrays(new double [][]
        { 
            new double[] { 0.0f, 0.0f, 0.0f },
            new double[] { 0.0f, 0.0f, 0.0f },
            new double[] { 0.0f, 0.0f, 0.0f }
        });
        for(int i = 0; i < 3; i++)
        {
            tcotis += System.Math.Max(0.0, evd.EigenValues.At(i).Real) * MathUtility.M(evd.EigenVectors.Column(i));
        }

        return tcotis;
    }

    public Matrix<double> CompressiveComponentOfTotalInternalStress(double dilation, double rigidity, double phi, double psi)
    {
        Evd<double> evd = TotalInternalStress(dilation, rigidity, phi, psi).Evd(Symmetricity.Symmetric);
        Matrix<double> ccotis = Matrix<double>.Build.DenseOfRowArrays(new double[][]
        {
            new double[] { 0.0f, 0.0f, 0.0f },
            new double[] { 0.0f, 0.0f, 0.0f },
            new double[] { 0.0f, 0.0f, 0.0f }
        });
        for (int i = 0; i < 3; i++)
        {
            ccotis += System.Math.Min(0.0, evd.EigenValues.At(i).Real) * MathUtility.M(evd.EigenVectors.Column(i));
        }

        return ccotis;
    }

    public Vector<double> TensileForce(double dilation, double rigidity, double phi, double psi, int node_index)
    {
        Matrix<double> tcotis = TensileComponentOfTotalInternalStress(dilation, rigidity, phi, psi);
        Vector<double> tf = Vector<double>.Build.DenseOfArray(new double[] { 0.0, 0.0, 0.0});
        double volume = Volume();
        Matrix<double> beta = Beta();

        for(int nodePosition = 0; nodePosition < 4; nodePosition++)
        {
            double scalar = 0.0f;
            for(int k = 0; k < 3; k++)
            {
                for(int l = 0; l < 3; l++)
                {
                    scalar += beta.At(nodePosition, l) * beta.At(node_index, k) * tcotis.At(k, l);
                }
            }
            Vector<double> node_world_pos = Vector<double>.Build.DenseOfArray(new double[] 
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

    public Vector<double> CompressiveForce(double dilation, double rigidity, double phi, double psi, int node_index)
    {
        Matrix<double> ccotis = CompressiveComponentOfTotalInternalStress(dilation, rigidity, phi, psi);
        Vector<double> tf = Vector<double>.Build.DenseOfArray(new double[] { 0.0, 0.0, 0.0 });
        double volume = Volume();
        Matrix<double> beta = Beta();

        for (int nodePosition = 0; nodePosition < 4; nodePosition++)
        {
            double scalar = 0.0f;
            for (int k = 0; k < 3; k++)
            {
                for (int l = 0; l < 3; l++)
                {
                    scalar += beta.At(nodePosition, l) * beta.At(node_index, k) * ccotis.At(k, l);
                }
            }
            Vector<double> node_world_pos = Vector<double>.Build.DenseOfArray(new double[]
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

    public void UpdateSeparationTensorOfNodes(double dilation, double rigidity, double phi, double psi)
    {
        for(int i = 0; i < 4; i++)
        {
            nodes[i].AddTensileForce(TensileForce(dilation, rigidity, phi, psi, i));
            nodes[i].AddCompressiveForce(CompressiveForce(dilation, rigidity, phi, psi, i));
        }
    }
}

public class FractureSimulator : MonoBehaviour
{
    [SerializeField]
    int Subdivisions;

    [SerializeField]
    float EdgeLengthUntilSubdivision;

    Mesh mesh;
    Vector3[] vertices = new Vector3[8];

    List<Cuboid> allcuboids = new List<Cuboid>();
    List<Tetrahedron> alltetrahedra = new List<Tetrahedron>();
    List<Node> allnodes = new List<Node>();

    public const double GLASS_DILATION = 1.04e8;
    public const double GLASS_RIGIDITY = 1.04e8;
    public const double GLASS_PSI = 0.0;
    public const double GLASS_PHI = 6760.0;
    public const double GLASS_DENSITY = 2588.0;
    public const double GLASS_TOUGHNESS = 10140.0;

    void Update()
    {
        foreach(Node n in allnodes)
        {
            // update world_pos and world_speed of each Node
            n.world_speed = (n.new_world_pos - n.world_pos) / Time.deltaTime;
        }

        foreach(Tetrahedron t in alltetrahedra)
        {
            t.UpdateSeparationTensorOfNodes(GLASS_DILATION, GLASS_RIGIDITY, GLASS_PHI, GLASS_PSI);
            // update separation tensor of each node
        }

        foreach (Node n in allnodes)
        {
            // check if crack occurs at node
            if (n.DoesCrackOccur(GLASS_TOUGHNESS /*glass*/))
            {
                // if yes, proceed with remeshing ...

            }
        }

        foreach (Tetrahedron t in alltetrahedra)
        {
            t.DrawEdges();
        }
    }

    void Start()
    {
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
        // TODO check for dimensional subdivision
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

        Debug.Log("There are " + allnodes.Count + " Nodes and " + alltetrahedra.Count + " Tetrahedra.");
    }
}
