using UnityEngine;

public class Deformer : MonoBehaviour
{
    GameObject fem_mesh;

    [SerializeField]
    bool stretch_x;
    [SerializeField]
    bool stretch_y;
    [SerializeField]
    bool stretch_z;

    

    void Start()
    {
        fem_mesh = GameObject.Find("FEM_Mesh");
    }

    void Update()
    {
        GameObject[] all_nodes = GameObject.FindGameObjectsWithTag("FEM_Node");
        foreach(GameObject node in all_nodes)
        {
            if(stretch_x)
            {
                if(node.transform.position.x > 0.0f)
                { 
                    node.transform.position = new Vector3(node.transform.position.x + 0.1f * Time.deltaTime, node.transform.position.y, node.transform.position.z);
                }
                else
                {
                    node.transform.position = new Vector3(node.transform.position.x - 0.1f * Time.deltaTime, node.transform.position.y, node.transform.position.z);
                }
            }
            if (stretch_y)
            {
                if (node.transform.position.y > 0.0f)
                {
                    node.transform.position = new Vector3(node.transform.position.x, node.transform.position.y + 0.1f * Time.deltaTime, node.transform.position.z);
                }
                else
                {
                    node.transform.position = new Vector3(node.transform.position.x, node.transform.position.y - 0.1f * Time.deltaTime, node.transform.position.z);
                }
            }
            if (stretch_z)
            {
                if (node.transform.position.z > 0.0f)
                {
                    node.transform.position = new Vector3(node.transform.position.x, node.transform.position.y, node.transform.position.z + 0.1f * Time.deltaTime);
                }
                else
                {
                    node.transform.position = new Vector3(node.transform.position.x, node.transform.position.y, node.transform.position.z - 0.1f * Time.deltaTime);
                }
            }
        }
    }
}
