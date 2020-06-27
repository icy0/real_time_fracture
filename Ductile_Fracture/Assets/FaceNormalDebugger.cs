using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[ExecuteInEditMode]
public class FaceNormalDebugger : MonoBehaviour
{
    private Mesh mesh;

    private Vector3[] points_on_triangles;
    void Start()
    {
        mesh = GetComponent<MeshFilter>().sharedMesh;
        points_on_triangles = new Vector3[4];

        for (int i = 0; i < mesh.triangles.Length/3; i ++)
        {
            Vector3[] vertices = new Vector3[3];
            vertices[0] = mesh.vertices[mesh.triangles[i*3]];
            vertices[1] = mesh.vertices[mesh.triangles[(i*3) + 1]];
            vertices[2] = mesh.vertices[mesh.triangles[(i*3) + 2]];

            points_on_triangles[i] = vertices[0] + ((vertices[1] - vertices[0]) * 0.5f) + ((vertices[2] - vertices[1]) * 0.3f);
            Debug.Log(vertices[0].ToString() + '\n' + vertices[1].ToString() + '\n' + vertices[2].ToString());
        }
    }

    void Update()
    {
        for(int i = 0; i < points_on_triangles.Length; i++)
        {
            Debug.DrawRay(points_on_triangles[i], mesh.normals[i], Color.blue);
        }
    }
}
