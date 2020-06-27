using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[ExecuteInEditMode]
public class HierarchyManager : MonoBehaviour
{
    public void UpdateHierarchy()
    {
        GameObject[] nodes = GameObject.FindGameObjectsWithTag("FEM_Node");
        foreach(GameObject node in nodes)
        {
            node.transform.SetParent(transform);
        }

        GameObject[] tetrahedra = GameObject.FindGameObjectsWithTag("FEM_Tetrahedron");
        foreach (GameObject tetrahedron in tetrahedra)
        {
            tetrahedron.transform.SetParent(transform);
        }
    }
}
