using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class Node
{
    public Vector3 pos;
    public Node(Vector3 p_pos)
    {
        pos = p_pos;
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

    public List<Vector3> GetPositions()
    {
        List <Vector3> list = new List<Vector3>();
        foreach(Node n in nodes)
        {
            list.Add(n.pos);
        }

        return list;
    }

    public void DrawEdges()
    {
        Debug.DrawLine(nodes[0].pos, nodes[1].pos);
        Debug.DrawLine(nodes[0].pos, nodes[2].pos);
        Debug.DrawLine(nodes[0].pos, nodes[3].pos);
        Debug.DrawLine(nodes[1].pos, nodes[2].pos);
        Debug.DrawLine(nodes[1].pos, nodes[3].pos);
        Debug.DrawLine(nodes[2].pos, nodes[3].pos);
    }

    public List<Tetrahedron> Subdivide()
    {
        List<Tetrahedron> tetrahedra = new List<Tetrahedron>();

        Node n5 = new Node(nodes[1].pos + (0.5f * (nodes[0].pos - nodes[1].pos)));
        tetrahedra.Add(new Tetrahedron(nodes[0], nodes[2], nodes[3], n5));
        tetrahedra.Add(new Tetrahedron(nodes[1], nodes[2], nodes[3], n5));

        return tetrahedra;
    }

    public void SetNewNode(int i, Node n)
    {
        nodes[i] = n;
    }
}

public class FractureSimulator : MonoBehaviour
{
    [SerializeField]
    int Subdivisions;

    Mesh mesh;
    Vector3[] vertices;

    [SerializeField]
    Transform DotPrefab;

    List<Tetrahedron> tetrahedra;
    List<Node> allnodes;

    void Start()
    {
        BuildFEM();
    }

    void Update()
    {
        foreach (Tetrahedron t in tetrahedra)
        {
            t.DrawEdges();
        }
    }

    private void BuildFEM()
    {
        // retrieve local vertex positions
        mesh = gameObject.GetComponent<MeshFilter>().mesh;
        vertices = mesh.vertices.Distinct().ToArray();

        // make nodes from these vertices
        Node[] nodes = new Node[8];
        for (int i = 0; i < 8; i++)
        {
            nodes[i] = new Node(vertices[i]);
        }

        // create 5 tetrahedra from the vertices that make up our cuboid
        tetrahedra = new List<Tetrahedron>();
        tetrahedra.Add(new Tetrahedron(nodes[2], nodes[3], nodes[1], nodes[5]));
        tetrahedra.Add(new Tetrahedron(nodes[5], nodes[7], nodes[6], nodes[1]));
        tetrahedra.Add(new Tetrahedron(nodes[4], nodes[5], nodes[6], nodes[2]));
        tetrahedra.Add(new Tetrahedron(nodes[5], nodes[2], nodes[1], nodes[6]));
        tetrahedra.Add(new Tetrahedron(nodes[0], nodes[1], nodes[2], nodes[6]));

        // subdivide the tetrahedra into more tetrahedra
        for (int i = 0; i < Subdivisions; i++)
        {
            int tetrahedraCount = tetrahedra.Count;
            for(int j = 0; j < tetrahedraCount; j++)
            {
                Tetrahedron current = tetrahedra[j];
                tetrahedra.RemoveAt(j);
                tetrahedra.AddRange(current.Subdivide());
            }
        }

        // make a list of all nodes
        allnodes = new List<Node>();
        foreach(Tetrahedron t in tetrahedra)
        {
            allnodes.Add(t.GetNodes()[0]);
            allnodes.Add(t.GetNodes()[1]);
            allnodes.Add(t.GetNodes()[2]);
            allnodes.Add(t.GetNodes()[3]);
        }

        // remove duplicates from that list
        for(int i = 0; i < allnodes.Count; i++)
        {
            for(int j = 0; j < allnodes.Count; j++)
            {
                if(i != j && allnodes[i].pos == allnodes[j].pos)
                {
                    allnodes.RemoveAt(j);
                    j--;
                }
            }
        }

        // if tetrahedra lost nodes due to duplicate eradication, update it
        foreach(Tetrahedron t in tetrahedra)
        {
            foreach(Node n in allnodes)
            {
                for(int i = 0; i < t.GetNodes().Length; i++)
                {
                    if(t.GetNodes()[i].pos == n.pos)
                    {
                        t.SetNewNode(i, n);
                    }
                }
            }
        }

        // make the nodes non-local / global
        foreach(Node n in allnodes)
        {
            n.pos += new Vector3(0.0f, 0.5f, 0.0f);
        }

        Debug.Log("There are " + allnodes.Count + " Nodes and " + tetrahedra.Count + " Tetrahedra.");
    }
}
