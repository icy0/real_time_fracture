using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RelationManager : MonoBehaviour
{
    private void Start()
    {
        UpdateRelations();
    }

    public void UpdateRelations()
    {
        GameObject[] node_gos = GameObject.FindGameObjectsWithTag("FEM_Node");
        GameObject[] tetrahedron_gos = GameObject.FindGameObjectsWithTag("FEM_Tetrahedron");

        foreach(GameObject node_go in node_gos)
        {
            Node node = node_go.GetComponent<Node>();
            int length_of_attached_elements = node.attached_elements.Count;
            for(int attached_element = 0; attached_element < length_of_attached_elements; attached_element++)
            {
                if(node.attached_elements[attached_element] == null)
                {
                    node.attached_elements.RemoveAt(attached_element);
                    attached_element--;
                    length_of_attached_elements--;
                }
            }
        }

        for (int tetrahedron_index = 0; tetrahedron_index < tetrahedron_gos.Length; tetrahedron_index++)
        {
            GameObject tetrahedron_go = tetrahedron_gos[tetrahedron_index];
            Tetrahedron tetrahedron = tetrahedron_go.GetComponent<Tetrahedron>();
            for (int node_index = 0; node_index < tetrahedron.node_transforms.Count; node_index++)
            {
                Transform node_t = tetrahedron.node_transforms[node_index];
                Node node = node_t.GetComponent<Node>();
                if(!node.attached_elements.Contains(tetrahedron))
                {
                    node.attached_elements.Add(tetrahedron);
                }
                tetrahedron.nodes[node_index] = node;
            }
        }

        for (int node_index = 0; node_index < node_gos.Length; node_index++)
        {
            GameObject node_go = node_gos[node_index];
            Node node = node_go.GetComponent<Node>();
            List<Tetrahedron> attached_elements = node.attached_elements;

            for (int outer_attached_element_index = 0; outer_attached_element_index < node.attached_elements.Count; outer_attached_element_index++)
            {
                for (int inner_attached_element_index = 0; inner_attached_element_index < node.attached_elements.Count; inner_attached_element_index++)
                {
                    if (inner_attached_element_index == outer_attached_element_index) continue;
                    if (!attached_elements[outer_attached_element_index].neighbors.ContainsKey(attached_elements[inner_attached_element_index]))
                    {
                        attached_elements[outer_attached_element_index].AddNeighbor(attached_elements[inner_attached_element_index], node);
                    }
                }
            }
        }
    }
}
