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
    Material tetrahedron_material = null;
    GameObject fracture_plane_prefab = null;

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
        tetrahedron_material = (Material)EditorGUI.ObjectField(new Rect(3, 100, position.width - 6, 20), "Tetrahedron Material", tetrahedron_material, typeof(Material));
        if(tetrahedron_material != null)
        {
            TetrahedronBuilder.tet_material = tetrahedron_material;
        }
        fracture_plane_prefab = (GameObject)EditorGUI.ObjectField(new Rect(3, 120, position.width - 6, 20), "Fracture Plane", fracture_plane_prefab, typeof(GameObject));
        if (fracture_plane_prefab != null)
        {
        }

        if (GUILayout.Button("Generate Tetrahedron"))
        {
            GameObject[] selected_gameobjects = Selection.gameObjects;
            if (selected_gameobjects.Length == 4)
            {
                GameObject.Find("FEM_Mesh").GetComponent<TetrahedronBuilder>().GenerateTetrahedron(selected_gameobjects);
                GameObject.Find("FEM_Mesh").GetComponent<RelationManager>().UpdateRelations();
            }
        }
        if (GUILayout.Button("Invert current Tetrahedra"))
        {
            GameObject[] selected_gameobjects = Selection.gameObjects;
            foreach (GameObject selected_gameobject in selected_gameobjects)
            {
                Mesh mesh = selected_gameobject.GetComponent<MeshFilter>().sharedMesh;
                int[] triangles = mesh.triangles;
                for (int i = 0; i < triangles.Length; i += 3)
                {
                    int temp = triangles[i + 1];
                    triangles[i + 1] = triangles[i + 2];
                    triangles[i + 2] = temp;
                }

                mesh.triangles = triangles;
                mesh.RecalculateNormals();
            }
        }
        if (GUILayout.Button("Update Relations"))
        {
            GameObject.Find("FEM_Mesh").GetComponent<RelationManager>().UpdateRelations();
        }
        if (GUILayout.Button("Spawn Fracture Plane at Node"))
        {
            GameObject[] selected_gameobjects = Selection.gameObjects;
            if (selected_gameobjects.Length == 1 && selected_gameobjects[0].tag == "FEM_Node")
            {
                GameObject fracture_plane_previsual = Instantiate(fracture_plane_prefab, selected_gameobjects[0].transform.position, Quaternion.identity);
                fracture_plane_previsual.name = "Fracture_Plane";
            }
        }
    }
}
