using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class test : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
        PhysicsBody physicsBody = GetComponent<PhysicsBody>();

        physicsBody.velocity = new Vector3(0, 0, -3);
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
