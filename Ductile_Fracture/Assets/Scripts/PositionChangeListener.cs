using System.Collections.Generic;
using UnityEngine;

[ExecuteInEditMode]
public class PositionChangeListener : MonoBehaviour
{
    Vector3 prev_position;
    static public List<IPositionChangeListener> listeners = new List<IPositionChangeListener>();

    void Start()
    {
        prev_position = transform.position;
    }

    void Update()
    {
        if(transform.position != prev_position)
        {
            foreach(IPositionChangeListener listener in listeners)
            {
                listener.UpdateListener(gameObject);
            }
        }
    }
}
