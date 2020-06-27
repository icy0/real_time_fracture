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

    private void UpdateRelations()
    {
        GameObject[] node_gos = GameObject.FindGameObjectsWithTag("FEM_Node");
        GameObject[] tetrahedron_gos = GameObject.FindGameObjectsWithTag("FEM_Tetrahedron");

        for (int tetrahedron_index = 0; tetrahedron_index < tetrahedron_gos.Length; tetrahedron_index++)
        {
            GameObject tetrahedron_go = tetrahedron_gos[tetrahedron_index];
            Tetrahedron tetrahedron = tetrahedron_go.GetComponent<Tetrahedron>();
            for (int node_index = 0; node_index < tetrahedron.node_transforms.Count; node_index++)
            {
                Transform node_t = tetrahedron.node_transforms[node_index];
                Node node = node_t.GetComponent<Node>();
                node.attached_elements.Add(tetrahedron);
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
                    if (!attached_elements[outer_attached_element_index].neighbors.ContainsKey(attached_elements[inner_attached_element_index])
                        && !attached_elements[outer_attached_element_index].Equals(attached_elements[inner_attached_element_index]))
                    {
                        attached_elements[outer_attached_element_index].AddNeighbor(attached_elements[inner_attached_element_index], node);
                    }
                }
            }
        }
    }

    private bool RayTriangleIntersection(Vector3 ray_origin, Vector3 ray_direction, Vector3[] triangle_vertices)
    {
        Debug.Assert(triangle_vertices.Length == 3);

        Vector3 e1 = triangle_vertices[1] - triangle_vertices[0];
        Vector3 e2 = triangle_vertices[2] - triangle_vertices[0];
        Vector3 q = Vector3.Cross(ray_direction, e2);

        float a = Vector3.Dot(e1, q);

        if (a > -float.Epsilon && a < float.Epsilon) // a == 0.0f?
            return false;

        float f = 1 / a;
        Vector3 s = ray_origin - triangle_vertices[0];
        float u = f * (Vector3.Dot(s, q));

        if (u < 0.0f)
            return false;

        Vector3 r = Vector3.Cross(s, e1);
        float v = f * (Vector3.Dot(ray_direction, r));

        if ((v < 0.0f) || ((u + v) > 1.0f))
            return false;

        return true;
    }

    private Vector3 FindTriangleNormal(Vector3[] triangle_vertices)
    {
        Debug.Assert(triangle_vertices.Length == 3);
        return Vector3.Normalize(Vector3.Cross(triangle_vertices[1] - triangle_vertices[0], triangle_vertices[2] - triangle_vertices[0]));
        // POTENTIAL ERROR this does not necessarily yield the correct normal direction
        // Fast workaround implemented.
    }

    private void BuildTetrahedron(ref Vector3[] vertices, out int[] vertex_indices, out Vector3[] normals)
    {
        Debug.Assert(vertices.Length == 4);

        Vector3[][] triangles = new Vector3[][] { 
            new Vector3[] { vertices[0], vertices[1], vertices[2] }, 
            new Vector3[] { vertices[0], vertices[3], vertices[1] }, 
            new Vector3[] { vertices[0], vertices[2], vertices[3] }, 
            new Vector3[] { vertices[3], vertices[2], vertices[1] } };

        vertex_indices = new int[] { 0, 1, 2, 0, 3, 1, 0, 2, 3, 3, 2, 1};
        
        normals = new Vector3[] { 
            FindTriangleNormal(triangles[0]), 
            FindTriangleNormal(triangles[1]), 
            FindTriangleNormal(triangles[2]), 
            FindTriangleNormal(triangles[3]) };

        for (int i = 0; i < 4; i++)
        {
            Vector3[] triangle = triangles[i];
            Vector3 some_point_on_current_triangle = triangle[0] + ((triangle[1] - triangle[0]) * 0.5f) + ((triangle[2] - triangle[1]) * 0.3f);

            if (RayTriangleIntersection(some_point_on_current_triangle, normals[i], triangles[(i + 1) % 4])
                || RayTriangleIntersection(some_point_on_current_triangle, normals[i], triangles[(i + 2) % 4])
                || RayTriangleIntersection(some_point_on_current_triangle, normals[i], triangles[(i + 3) % 4]))
            {
                int temp = vertex_indices[(i*3) + 1];
                vertex_indices[(i * 3) + 1] = vertex_indices[(i * 3) + 2];
                vertex_indices[(i * 3) + 2] = temp;
                normals[i] = -normals[i];
            }
        }
    }

    private GameObject GenerateTetrahedron(GameObject[] selected_gameobjects)
    {
        GameObject tetrahedron_go = new GameObject("Tetrahedron");
        Mesh tetrahedron_mesh = new Mesh();
        Vector3[] vertices = new Vector3[4];
        for (int i = 0; i < 4; i++)
        {
            vertices[i] = selected_gameobjects[i].transform.position;
        }

        int[] vertex_indices;
        Vector3[] normals;
        BuildTetrahedron(ref vertices, out vertex_indices, out normals);

        tetrahedron_mesh.vertices = vertices;
        tetrahedron_mesh.triangles = vertex_indices;
        tetrahedron_mesh.normals = normals;
        tetrahedron_go.AddComponent<MeshFilter>().mesh = tetrahedron_mesh;
        MeshRenderer mesh_renderer = tetrahedron_go.AddComponent<MeshRenderer>();
        mesh_renderer.sharedMaterial = selected_gameobjects[0].GetComponent<MeshRenderer>().sharedMaterial;
        mesh_renderer.material.color = Color.green;
        tetrahedron_go.tag = "FEM_Tetrahedron";

        Tetrahedron tetrahedron = tetrahedron_go.AddComponent<Tetrahedron>();
        for (int i = 0; i < 4; i++)
        {
            tetrahedron.AddNode(selected_gameobjects[i].transform);
        }

        //tetrahedron.AddComponent<FaceNormalDebugger>();
        return tetrahedron_go;
    }

    private void OnGUI()
    {
        if (GUILayout.Button("Generate Tetrahedron"))
        {
            GameObject[] selected_gameobjects = Selection.gameObjects;
            if(selected_gameobjects.Length == 4)
            {
                GenerateTetrahedron(selected_gameobjects);
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
            UpdateRelations();
        }
    }
}
