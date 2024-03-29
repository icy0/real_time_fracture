﻿using System;
using MathNet.Numerics.LinearAlgebra;
using UnityEngine;

public class MathUtility
{
    public static int KroneckerDelta(int i, int j)
    {
        if (i == j) return 1;
        else return 0;
    }

    // Vector conversion from one library format into another
    public static Vector<float> ToVector(Vector3 x)
    {
        Debug.Assert(x != null);

        return Vector<float>.Build.DenseOfArray(new float[] { x.x, x.y, x.z });
    }

    // Projects vector a onto vector b
    public static Vector<float> ProjectOnto(Vector<float> a, Vector<float> b)
    {
        Vector<float> projection = Vector<float>.Build.DenseOfArray(new float[] { 0.0f, 0.0f, 0.0f });

        projection = (a.DotProduct(b) / b.DotProduct(b)) * b;

        return projection;
    }

    // an element-wise comparison with a certain tolerance
    public static bool EqualsRoughly(Vector<float> x, Vector<float> y, float maximum_difference)
    {
        Debug.Assert(x != null);
        Debug.Assert(y != null);

        for (int member = 0; member < 3; member++)
        {
            
            //if (x.At(member) - y.At(member) > maximum_difference)
            if (Math.Abs(x.At(member) - y.At(member)) > maximum_difference)
            {
                return false;
            }
        }
        return true;
    }

    public static Tuple<bool /*intersection?*/, bool /*line segment within plane?*/, Vector<float>> LinePlaneIntersection(Vector<float> point_on_plane, Vector<float> plane_normal, Vector<float> a, Vector<float> b)
    {
        Debug.Assert(point_on_plane != null);
        Debug.Assert(plane_normal != null);
        Debug.Assert(a != null);
        Debug.Assert(b != null);

        // if the line segment is parallel to the plane, it either lies within the plane or there is no point of intersection
        if ((a - b).DotProduct(plane_normal) == 0.0f)
        {
            // if the line segment lies within the plane, point a lies on the plane
            if ((point_on_plane - a).DotProduct(plane_normal) == 0.0f)
            {
                return new Tuple<bool, bool, Vector<float>>(true, true, null);
            }
            // else there is no point of intersection
            return new Tuple<bool, bool, Vector<float>>(false, false, null);
        }

        Vector<float> w = a - point_on_plane;
        Vector<float> u = b - a;
        float length_of_edge = (float)u.L2Norm();
        float scalar = (-plane_normal).DotProduct(w) / plane_normal.DotProduct(u);

        if (scalar >= 0.0f && scalar <= 1.0f)
        {
            return new Tuple<bool, bool, Vector<float>>(true, false, a + (u * scalar));
        }
        // no point of intersection
        return new Tuple<bool, bool, Vector<float>>(false, false, null);
    }

    // this function checks whether a node n is in the positive halfspace of a given plane or not
    public static bool IsOnPositiveSide(Node n, Vector<float> point_on_plane, Vector<float> plane_normal)
    {
        Debug.Assert(n != null);
        Debug.Assert(point_on_plane != null);
        Debug.Assert(plane_normal != null);

        Vector<float> n_pos = Vector<float>.Build.DenseOfArray(new float[] { n.transform.position.x, n.transform.position.y, n.transform.position.z });

        if (plane_normal.DotProduct(n_pos - point_on_plane) > 0)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    // this function is given 2 nodes which define a line and two other nodes x and y. This
    // function then proceeds to find out which of x and y lies on the line spun between the first 2 nodes and returns it.
    public static Node WhichNodeIsOnLine(Node line_n1, Node line_n2, Node intersection_x, Node intersection_y)
    {
        Debug.Assert(line_n1 != null);
        Debug.Assert(line_n2 != null);
        Debug.Assert(intersection_x != null);
        Debug.Assert(intersection_y != null);

        Vector<float> n1 = Vector<float>.Build.DenseOfArray(new float[] { line_n1.transform.position.x, line_n1.transform.position.y, line_n1.transform.position.z });
        Vector<float> n2 = Vector<float>.Build.DenseOfArray(new float[] { line_n2.transform.position.x, line_n2.transform.position.y, line_n2.transform.position.z });
        Vector<float> x = Vector<float>.Build.DenseOfArray(new float[] { intersection_x.transform.position.x, intersection_x.transform.position.y, intersection_x.transform.position.z });
        Vector<float> y = Vector<float>.Build.DenseOfArray(new float[] { intersection_y.transform.position.x, intersection_y.transform.position.y, intersection_y.transform.position.z });

        Vector<float> projection_of_x = ProjectOnto(x - n1, n2 - n1) - (x - n1);
        Vector<float> projection_of_y = ProjectOnto(y - n1, n2 - n1) - (y - n1);

        return (projection_of_x.L2Norm() < projection_of_y.L2Norm()) ? intersection_x : intersection_y;
    }

    // returns the index of a node n of a tetrahedron t
    public static int GetIndexOf(Tetrahedron x, Node n)
    {
        Debug.Assert(x != null);
        Debug.Assert(n != null);

        Node[] nodes = x.nodes;
        int index = -1;

        for (int i = 0; i < 4; i++)
        {
            if (nodes[i].Equals(n))
            {
                return index = i;
            }
        }

        return index;
    }

    // produces a threedimensional representation of the kronecker delta
    public static Vector<float> KroneckerDeltaVector(int i)
    {
        Debug.Assert((i > 0) && (i < 4), "indexing error, easy to fix.");
        Vector<float> kdv = Vector<float>.Build.DenseOfArray(new float[] { 0.0f, 0.0f, 0.0f, 0.0f });
        kdv.At(i - 1, 1.0f);

        return kdv;
    }

    // this function produces a matrix, which has the length of a given vector a as its only eigenvalue.
    public static Matrix<float> M(Vector<float> a)
    {
        if (!(a.At(0) == 0.0 && a.At(1) == 0.0 && a.At(2) == 0.0))
        {
            Matrix<float> m = Matrix<float>.Build.DenseOfRowArrays(new float[][]
            {
               new float[] { a.At(0) * a.At(0), a.At(0) * a.At(1), a.At(0) * a.At(2)},
               new float[] { a.At(1) * a.At(0), a.At(1) * a.At(1), a.At(1) * a.At(2)},
               new float[] { a.At(2) * a.At(0), a.At(2) * a.At(1), a.At(2) * a.At(2)}
            });

            return m / (float) a.L2Norm();
        }
        else
        {
            return Matrix<float>.Build.DenseOfRowArrays(new float[][]
            {
               new float[] { 0.0f, 0.0f, 0.0f },
               new float[] { 0.0f, 0.0f, 0.0f },
               new float[] { 0.0f, 0.0f, 0.0f }
            });
        }
    }
}
