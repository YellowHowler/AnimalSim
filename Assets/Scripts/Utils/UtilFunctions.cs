using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public static class UtilFunctions
{
    public static T GetOrAddComponent<T>(this GameObject go) where T : Component
    {
        //Attempt to find the component on the object
        T newComponent = go.GetComponent<T>();

        //If it doesn't exist, create a new one
        if (newComponent == null)
        {
            newComponent = go.AddComponent<T>();
        }

        //Return the component
        return newComponent;
    }

    public static int NumChildrenWithComponent<T>(this Transform parent) where T : Component
    {
        int count = 0;
        for (int i = 0; i < parent.childCount; i++)
        {
            Transform child = parent.GetChild(i);
            if (child.GetComponent<T>() != null)
            {
                count++;
            }
        }
        return count;
    }

    // Clamp a vector to have at least a minimum magnitude
    public static Vector3 ClampMagnitude(Vector3 v, float minMagnitude = -Mathf.Infinity, float maxMagnitude = Mathf.Infinity)
    {
        float mag = v.magnitude;

        if (mag < minMagnitude && mag > 0f)
        {
            return v.normalized * minMagnitude;
        }
        if (v.sqrMagnitude > maxMagnitude * maxMagnitude)
        {
            return v.normalized * maxMagnitude;
        }

        return v;
    }

    public static Vector2 Vec2GetComponent(Vector2 v, Vector2 dir)
    {
        Vector2 projection = Vector2.Dot(v, dir.normalized) * dir.normalized;
        return projection;
    } 

    public static Vector3 Vec3GetComponent(Vector3 v, Vector2 dir)
    {
        Vector3 projection = Vector3.Dot(v, dir.normalized) * dir.normalized;
        return projection;
    } 
}
