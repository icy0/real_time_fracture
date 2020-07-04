using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
using UnityEditor.Events;
using UnityEditor.SceneManagement;
using UnityEngine.UIElements;
using MathNet.Numerics.LinearAlgebra.Complex;

public class FractureTool : EditorWindow, IPositionChangeListener
{
    [MenuItem("Window/Fracture Tool")]
    static void OpenWindow()
    {
        FractureTool tool = (FractureTool)GetWindow(typeof(FractureTool));
        tool.Show();
    }

    private void OnEnable()
    {
        PositionChangeListener.listeners.Add(this);
    }

    private void Update()
    {

    }

    public void UpdateListener(GameObject x)
    {
        GameObject[] tetrahedra = GameObject.FindGameObjectsWithTag("FEM_Tetrahedron");
        for(int i = 0; i < tetrahedra.Length; i++)
        {
            List<Transform> attached_nodes = tetrahedra[i].GetComponent<Tetrahedron>().node_transforms;
            for(int j = 0; j < attached_nodes.Count; j++)
            {
                if(attached_nodes[j] == x.transform)
                {
                    UpdateTetrahedron(tetrahedra[i], x.transform);
                }
            }
        }
    }

    private void UpdateTetrahedron(GameObject tetrahedron, Transform node)
    {
        int index_of_node = tetrahedron.GetComponent<Tetrahedron>().node_transforms.IndexOf(node);
        MeshFilter mesh_filter = tetrahedron.GetComponent<MeshFilter>();
        Mesh mesh = mesh_filter.sharedMesh;

        Vector3[] vertices = mesh.vertices;
        vertices[index_of_node] = node.position;
        mesh.vertices = vertices;
        mesh.RecalculateNormals();
        mesh_filter.sharedMesh = mesh;
    }

    private void OnHierarchyChange()
    {
        GameObject.Find("FEM_Mesh").GetComponent<HierarchyManager>().UpdateHierarchy();
    }

    private void OnGUI()
    {
        if (GUILayout.Button("Generate Tetrahedron"))
        {
            GameObject[] selected_gameobjects = Selection.gameObjects;
            if(selected_gameobjects.Length == 4)
            {
                GameObject.Find("FEM_Mesh").GetComponent<TetrahedronBuilder>().GenerateTetrahedron(selected_gameobjects);
                GameObject.Find("FEM_Mesh").GetComponent<RelationManager>().UpdateRelations();
            }
        }
        if(GUILayout.Button("Invert current Tetrahedra"))
        {
            GameObject[] selected_gameobjects = Selection.gameObjects;
            foreach(GameObject selected_gameobject in selected_gameobjects)
            {
                Mesh mesh = selected_gameobject.GetComponent<MeshFilter>().sharedMesh;
                int[] triangles = mesh.triangles;
                for(int i = 0; i < triangles.Length; i += 3)
                {
                    int temp = triangles[i + 1];
                    triangles[i + 1] = triangles[i + 2];
                    triangles[i + 2] = temp;
                }

                mesh.triangles = triangles;
                mesh.RecalculateNormals();
            }
        }
        if(GUILayout.Button("Update Relations"))
        {
            GameObject.Find("FEM_Mesh").GetComponent<RelationManager>().UpdateRelations();
        }
    }
}
