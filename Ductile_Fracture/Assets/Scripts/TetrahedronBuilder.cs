using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TetrahedronBuilder : MonoBehaviour
{
    public static Material tet_material;
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

        vertex_indices = new int[] { 0, 1, 2, 0, 3, 1, 0, 2, 3, 3, 2, 1 };

        normals = new Vector3[] {
            FindTriangleNormal(triangles[0]),
            FindTriangleNormal(triangles[1]),
            FindTriangleNormal(triangles[2]),
            FindTriangleNormal(triangles[3]) };

        // we take the first triangle and find the node which is not on it

        //for (int i = 0; i < 4; i++)
        //{
        //    Vector3[] triangle = triangles[i];
        //    Vector3 some_point_on_current_triangle = triangle[0] + ((triangle[1] - triangle[0]) * 0.5f) + ((triangle[2] - triangle[1]) * 0.3f);

        //    if (RayTriangleIntersection(some_point_on_current_triangle, normals[i], triangles[(i + 1) % 4])
        //        || RayTriangleIntersection(some_point_on_current_triangle, normals[i], triangles[(i + 2) % 4])
        //        || RayTriangleIntersection(some_point_on_current_triangle, normals[i], triangles[(i + 3) % 4]))
        //    {
        //        int temp = vertex_indices[(i * 3) + 1];
        //        vertex_indices[(i * 3) + 1] = vertex_indices[(i * 3) + 2];
        //        vertex_indices[(i * 3) + 2] = temp;
        //        normals[i] = -normals[i];
        //    }
        //}
    }

    public GameObject GenerateTetrahedron(GameObject[] selected_gameobjects)
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
        //mesh_renderer.sharedMaterial = tet_material; 
        mesh_renderer.sharedMaterial = Resources.Load("Materials/Tetrahedron", typeof(Material)) as Material;
        tetrahedron_go.tag = "FEM_Tetrahedron";

        Tetrahedron tetrahedron = tetrahedron_go.AddComponent<Tetrahedron>();
        for (int i = 0; i < 4; i++)
        {
            tetrahedron.AddNode(selected_gameobjects[i].transform);
        }

        tetrahedron.gameObject.AddComponent<FaceNormalDebugger>();
        return tetrahedron_go;
    }
}
