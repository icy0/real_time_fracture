using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
using UnityEngine.UIElements;
using MathNet.Numerics.LinearAlgebra.Complex;

public class FractureTool : EditorWindow
{
    [MenuItem("Window/Fracture Tool")]
    static void OpenWindow()
    {
        FractureTool tool = (FractureTool)GetWindow(typeof(FractureTool));
        tool.Show();
    }

    private void OnEnable()
    {

    }

    private void OnGUI()
    {
        if (GUILayout.Button("Generate Tetrahedron"))
        {
            GameObject[] selected_gameobjects = Selection.gameObjects;
            if(selected_gameobjects.Length == 4)
            {
                GameObject tetrahedron = new GameObject("Tetrahedron");
                Mesh tetrahedron_mesh = new Mesh();
                Vector3[] vertices = new Vector3[4];
                for(int i = 0; i < 4; i++)
                {
                    vertices[i] = selected_gameobjects[i].transform.position;
                    // selected_gameobjects[i].transform.parent = tetrahedron.transform;
                }

                int[] vertex_indices = new int[] { 0, 1, 2, 0, 3, 1, 0, 2, 3, 3, 2, 1 };
                tetrahedron_mesh.vertices = vertices;
                tetrahedron_mesh.triangles = vertex_indices;
                tetrahedron_mesh.RecalculateNormals();
                tetrahedron.AddComponent<MeshFilter>().mesh = tetrahedron_mesh;
                tetrahedron.AddComponent<MeshRenderer>();
            }
        }
    }
}
