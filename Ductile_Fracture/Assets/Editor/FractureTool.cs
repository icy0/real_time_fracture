using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

public class FractureTool : EditorWindow, IPositionChangeListener
{
    /* opens the window which holds the 2 buttons to create tetrahedra */
    [MenuItem("Window/Fracture Tool")]
    static void OpenWindow()
    {
        FractureTool tool = (FractureTool)GetWindow(typeof(FractureTool));
        tool.Show();
    }

    private void OnEnable()
    {
        // register as a listener
        PositionChangeListener.listeners.Add(this);
    }

    /* inherited from the IPositionChangeListener-interface. It is triggered, when one NodePrefab-instance
     * changes its positon. It goes through all the NodePrefab-instances and updates the tetrahedra they are attached to. */
    public void UpdateListener(GameObject changed_node)
    {
        GameObject[] tetrahedra = GameObject.FindGameObjectsWithTag("FEM_Tetrahedron");
        // for each existing tetrahedron
        for(int i = 0; i < tetrahedra.Length; i++)
        {
            // for each node that is attached to the tetrahedron in question
            List<Transform> attached_nodes = tetrahedra[i].GetComponent<Tetrahedron>().node_transforms;
            for(int j = 0; j < attached_nodes.Count; j++)
            {
                // if the node, that has changed, is attached to the tetrahedron in question
                if(attached_nodes[j] == changed_node.transform)
                {
                    // update the tetrahedron in question
                    GameObject tetrahedron = tetrahedra[i];
                    Transform node = changed_node.transform;

                    int index_of_node = tetrahedron.GetComponent<Tetrahedron>().node_transforms.IndexOf(node);
                    MeshFilter mesh_filter = tetrahedron.GetComponent<MeshFilter>();
                    Mesh mesh = mesh_filter.sharedMesh;

                    Vector3[] vertices = mesh.vertices;
                    vertices[index_of_node] = node.position;
                    mesh.vertices = vertices;
                    mesh.RecalculateNormals();
                    mesh_filter.sharedMesh = mesh;
                }
            }
        }
    }

    /* callback, triggered if anything in the entire game object hierarchy changes.
     * callback active in edit mode as well as in play mode. */
    private void OnHierarchyChange()
    {
        GameObject fem_mesh = GameObject.Find("FEM_Mesh");
        GameObject[] nodes = GameObject.FindGameObjectsWithTag("FEM_Node");
        foreach (GameObject node in nodes)
        {
            node.transform.SetParent(fem_mesh.transform);
        }

        GameObject[] tetrahedra = GameObject.FindGameObjectsWithTag("FEM_Tetrahedron");
        foreach (GameObject tetrahedron in tetrahedra)
        {
            tetrahedron.transform.SetParent(fem_mesh.transform);
        }
    }

    /* immediate mode gui for the window that is opened with OpenWindow() */
    private void OnGUI()
    {
        if (GUILayout.Button("Generate Tetrahedron"))
        {
            GameObject[] selected_gameobjects = Selection.gameObjects;
            if (selected_gameobjects.Length == 4)
            {
                GameObject.Find("FEM_Mesh").GetComponent<TetrahedronBuilder>().GenerateTetrahedron(selected_gameobjects);
                GameObject.Find("FEM_Mesh").GetComponent<RelationManager>().UpdateRelations();
            }
        }
        if (GUILayout.Button("Update Relations"))
        {
            GameObject.Find("FEM_Mesh").GetComponent<RelationManager>().UpdateRelations();
        }
    }
}
