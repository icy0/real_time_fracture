﻿using System.Collections.Generic;
using System;
using UnityEngine;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Factorization;


public class FractureSimulator : MonoBehaviour
{
    RelationManager relation_manager;

    private class Material
    {
        public float dilation;
        public float rigidity;
        public float psi;
        public float phi;
        public float density;
        public float toughness;
        public float elastic_limit;
        public float plastic_limit;

        public Material(float dilation, float rigidity, float psi, float phi, float density, float toughness, float elastic_limit, float plastic_limit)
        {
            this.dilation = dilation;
            this.rigidity = rigidity;
            this.psi = psi;
            this.phi = phi;
            this.density = density;
            this.toughness = toughness;
            this.elastic_limit = elastic_limit;
            this.plastic_limit = plastic_limit;
        }
    }

    List<Tetrahedron> alltetrahedra = new List<Tetrahedron>();
    List<Node> allnodes = new List<Node>();

    static Material glass = new Material(1.04e8f, 1.04e8f, 0.0f, 6760.0f, 2588.0f, 10140.0f, 0.022f, 0.4f);
    static Material current = glass;

    void Start()
    {
        relation_manager = GameObject.Find("FEM_Mesh").GetComponent<RelationManager>();
        relation_manager.UpdateRelations();
        Time.fixedDeltaTime = 0.1f;
        GameObject[] nodes = GameObject.FindGameObjectsWithTag("FEM_Node");
        GameObject[] tetrahedra = GameObject.FindGameObjectsWithTag("FEM_Tetrahedron");

        for (int node_index = 0; node_index < nodes.Length; node_index++)
        {
            GameObject node_go = nodes[node_index];
            allnodes.Add(node_go.GetComponent<Node>());
        }

        for (int tetrahedron_index = 0; tetrahedron_index < tetrahedra.Length; tetrahedron_index++)
        {
            GameObject tetrahedron_go = tetrahedra[tetrahedron_index];
            alltetrahedra.Add(tetrahedron_go.GetComponent<Tetrahedron>());
        }

        Debug.Log("There are " + alltetrahedra.Count + " Tetrahedra and " + allnodes.Count + " Nodes.");

        // DEBUG CODE:
        foreach(GameObject n in nodes)
        {
            if (n.GetComponent<Node>().crack_at_start)
            {
                n.GetComponent<Node>().fracture_plane_normal = GetNormalFromPlane(GameObject.Find("Fracture_Plane"));
                Tuple<List<Tetrahedron>, List<Tetrahedron>, List<Node>> result = n.GetComponent<Node>().Crack(n.transform.parent.position);

                // update tetrahedra collections
                foreach(Tetrahedron t in result.Item1)
                {
                    alltetrahedra.Remove(t);
                    DestroyImmediate(t.gameObject);
                }
                alltetrahedra.AddRange(result.Item2);

                // update node collections
                allnodes.Remove(n.GetComponent<Node>());
                DestroyImmediate(n.gameObject);
                allnodes.AddRange(result.Item3);
                break;
            }
        }
    }

    Vector<float> GetNormalFromPlane(GameObject fracture_plane)
    {
        Vector3 normal = fracture_plane.transform.up;
        return Vector<float>.Build.DenseOfArray(new float[] { normal.x, normal.y, normal.z});
    }

    private void FixedUpdate()
    {
        // calculate velocity of each node
        foreach (Node n in allnodes)
        {
            n.velocity = (n.transform.position - n.old_world_position) / Time.deltaTime; // not sure if division by time or multiplication with time
            n.old_world_position = n.transform.position;
        }

        // update the internal forces of each tetrahedron
        foreach (Tetrahedron t in alltetrahedra)
        {
            t.UpdateInternalForcesOfNodes(current.dilation, current.rigidity, current.phi, current.psi, current.elastic_limit, current.plastic_limit);
        }

        List<Node> nodes_to_be_removed = new List<Node>();

        // check if any node exceeds its deformation limit
        int node_count = allnodes.Count;
        for (int i = 0; i < node_count; i++)
        {
            Node n = allnodes[i];
            if (n.DoesCrackOccur(current.toughness))
            {
                nodes_to_be_removed.Add(n);
                Tuple<List<Tetrahedron>, List<Tetrahedron>, List<Node>> updated_tets_and_nodes = n.Crack(transform.position);

                foreach (Tetrahedron t in updated_tets_and_nodes.Item1)
                {
                    alltetrahedra.Remove(t);
                    t.nodes = null;
                    t.node_transforms.Clear();
                    Destroy(t.gameObject);
                }
                alltetrahedra.AddRange(updated_tets_and_nodes.Item2);
                allnodes.AddRange(updated_tets_and_nodes.Item3);
                Debug.Log("Crack occurs");

                relation_manager.UpdateRelations();

                foreach (Tetrahedron t in alltetrahedra)
                {
                    t.Beta();
                }

                foreach(Tetrahedron t in updated_tets_and_nodes.Item2)
                {
                    t.ResetElasticStrain();
                    t.ResetPlasticStrain();
                }
            }
            n.ClearTensileAndCompressiveForces();
        }

        GameObject[] nodes = GameObject.FindGameObjectsWithTag("FEM_Node");
        foreach (GameObject n_go in nodes)
        {
            Node n = n_go.GetComponent<Node>();
            Debug.Assert(n != null);
            if (n.attached_elements.Count == 0)
            {
                nodes_to_be_removed.Add(n);
            }
        }

        for(int node = 0; node < nodes_to_be_removed.Count; node++)
        {
            allnodes.Remove(nodes_to_be_removed[node]);
            Destroy(nodes_to_be_removed[node].gameObject);
        }
    }

    void Update()
    {
    }
}
