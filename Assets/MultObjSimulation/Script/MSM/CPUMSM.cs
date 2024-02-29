using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;
using System.Text;
using System;
using System.Linq;
using UnityEngine.Rendering;
using PBD;
using Octree;

public class CPUMSM : MonoBehaviour
{
    
    private float dt = 0.005f; // have to devide by 20
    private Vector3 gravity = new Vector3(0f, -9.81f, 0f);
    
    private int speed = 1;
    
    private GameObject[] collidableObjects;

    private int nodeCount;
    
    private int triCount; // size of triangle
    

    
    private string modelName;

    private bool useInteriorSpring = false;
   
    private Vector3[] Positions;
    
    private Vector3[] Velocities;
    private Vector3[] Forces;
    private List<Triangle> triangles = new List<Triangle>();
    

    private List<int> initTrianglePtr = new List<int>();
    private List<Triangle> initTriangle = new List<Triangle>();

    private int[] triArray;
    private vertData[] vDataArray;
    private ComputeBuffer vertexBuffer;
    private ComputeBuffer triBuffer;

    private Shader renderingShader;
    private Color matColor;
    private Material material;

    

    private BoundingBox boudingBox;
    private BoundingBox floorBB;


    public void SetCollidableObj(GameObject[] CollidableObjects)
    {
        collidableObjects = CollidableObjects;
    }

    public void SetCoefficient( float Dt, Vector3 Gravity,
       int Speed)
    {
        
        dt = Dt;
        gravity = Gravity;
       
        speed = Speed;
    }


    public void SetMeshData(Vector3[] NodePositions, List<Triangle> MeshTriangle,
        int[] TriArray)
    {
        Positions = NodePositions;
        triangles = MeshTriangle;
        triArray = TriArray;
    }

    public void SetMeshData(string TetGenModel, bool UseInteriorSpring)
    {
        modelName = TetGenModel;
        useInteriorSpring = UseInteriorSpring;
    }
    public void SetRenderer(Shader RenderingShader, Color MatColor)
    {
        renderingShader = RenderingShader;
        matColor = MatColor;
    }

    public void SetBoundingBox(Vector3 min, Vector3 max)
    {
        boudingBox.min = min;
        boudingBox.max = max;
    }

    public int GetNodeCount()
    {
        return nodeCount;
    }

    public List<Triangle> GetTriCount()
    {
        return triangles;
    }

    public Vector3[] GetPosition()
    {
        return Positions;
    }

    public void StartObj()
    {
        FindFloorMinMax();
        transformMesh();


        material = new Material(renderingShader);
        material.color = matColor;

        material.SetBuffer(Shader.PropertyToID("vertsBuff"), vertexBuffer);
        material.SetBuffer(Shader.PropertyToID("triBuff"), triBuffer);

        vertexBuffer.SetData(vDataArray);
        triBuffer.SetData(triArray);

        Vector3 translation = transform.position;
        Vector3 scale = this.transform.localScale;
        Quaternion rotationeuler = transform.rotation;
        Matrix4x4 trs = Matrix4x4.TRS(translation, rotationeuler, scale);
        material.SetMatrix("TRSMatrix", trs);
        material.SetMatrix("invTRSMatrix", trs.inverse);

    }

    private void FindFloorMinMax()
    {
        Vector3[] vertices;

        vertices = collidableObjects[0].GetComponent<MeshFilter>().mesh.vertices;
        floorBB.min = collidableObjects[0].transform.TransformPoint(vertices[0]);
        floorBB.max = collidableObjects[0].transform.TransformPoint(vertices[0]);

        for (int i = 0; i < vertices.Length; i++)
        {
            Vector3 allVerts = collidableObjects[0].transform.TransformPoint(vertices[i]);

            floorBB.min.x = Mathf.Min(floorBB.min.x, allVerts.x);
            floorBB.min.y = Mathf.Min(floorBB.min.y, allVerts.y);
            floorBB.min.z = Mathf.Min(floorBB.min.z, allVerts.z);

            floorBB.max.x = Mathf.Max(floorBB.max.x, allVerts.x);
            floorBB.max.y = Mathf.Max(floorBB.max.y, allVerts.y);
            floorBB.max.z = Mathf.Max(floorBB.max.z, allVerts.z);
            floorBB.max.y += 0.01f;
        }
    }

    private void transformMesh()
    {
        string filePath = Application.dataPath + "/MultObjSimulation/TetModel/";
        LoadTetModel.LoadData(filePath + modelName, useInteriorSpring, this.gameObject);

        Positions = LoadTetModel.positions.ToArray();
        triangles = LoadTetModel.triangles;

        triArray = LoadTetModel.triangleArr.ToArray();
        
        nodeCount = Positions.Length;
        
        triCount = triangles.Count;
        Velocities = new Vector3[nodeCount];
        Forces = new Vector3[nodeCount];


        Velocities.Initialize();
        Forces.Initialize();

        vDataArray = new vertData[nodeCount];

        for (int i = 0; i < nodeCount; i++)
        {
            vDataArray[i] = new vertData();
            vDataArray[i].pos = Positions[i];
            vDataArray[i].norms = Vector3.zero;
            vDataArray[i].uvs = Vector3.zero;
        }

        int triBuffStride = sizeof(int);
        triBuffer = new ComputeBuffer(triArray.Length, triBuffStride, ComputeBufferType.Default);


        int vertsBuffstride = 8 * sizeof(float);
        vertexBuffer = new ComputeBuffer(vDataArray.Length, vertsBuffstride, ComputeBufferType.Default);

        initTrianglePtr.Add(0);
        for (int i = 0; i < nodeCount; i++)
        {
            foreach (Triangle tri in triangles)
            {
                if (tri.vertices[0] == i || tri.vertices[1] == i || tri.vertices[2] == i)
                    initTriangle.Add(tri);
            }
            initTrianglePtr.Add(initTriangle.Count);
        }

        LoadTetModel.ClearData();

        //print("node count: " + nodeCount);
        //print("springCounnt: " + springCount);
        //print("triCount: " + triCount);

    }

    private float calculateTriArea(Vector3 p1, Vector3 p2, Vector3 p3)
    {
        float area = 0.0f;
        float term1, term2, term3;

        term1 = (p2.y - p1.y) * (p3.z - p1.z) - (p2.z - p1.z) * (p3.y - p1.y);
        term2 = (p2.x - p1.x) * (p3.z - p1.z) - (p2.z - p1.z) * (p3.x - p1.x);
        term3 = (p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x);

        area = 0.5f * Mathf.Sqrt(term1 * term1 + term2 * term2 + term3 * term3);
        return area;
    }

    public void UpdateObj()
    {
        //for (int i = 0; i < speed; i++)
        {
            UpdateNodes();
            CheckCollisionWithFloor();
            UpdateReverseVelocity();

        }

        computeVertexNormal();
        vertexBuffer.SetData(vDataArray);

        Bounds bounds = new Bounds(Vector3.zero, Vector3.one * 10000);


        material.SetPass(0);
        Graphics.DrawProcedural(material, bounds, MeshTopology.Triangles, triArray.Length,
            1, null, null, ShadowCastingMode.On, true, gameObject.layer);
        if (Input.GetKey(KeyCode.Escape))
            UnityEditor.EditorApplication.isPlaying = false;

    }




    void UpdateNodes()
    {
        //Euler method
        for (int i = 0; i < nodeCount; i++)
        {

            Vector3 pos = Positions[i];
            Vector3 vel = Velocities[i];

            vel = vel + gravity * dt;
            pos = pos + vel * dt;

            Positions[i] = pos;
            Velocities[i] = vel;

            vDataArray[i].pos = Positions[i];
        }
    }

    private void UpdateReverseVelocity()
    {
        for (int i = 0; i < nodeCount; i++)
        {
            if (floorBB.collide)
            {
                Velocities[i] *= -1;
                Positions[i].y += .1f;
                vDataArray[i].pos = Positions[i];
            }
        }
    }

    private void CheckCollisionWithFloor()
    {
        floorBB.collide = Intersection.AABB(boudingBox.min, boudingBox.max,
            floorBB.min, floorBB.max);
    }

    void computeVertexNormal()
    {
        for (int i = 0; i < triCount; i++)
        {
            Vector3 v1 = Positions[triArray[i * 3 + 0]];
            Vector3 v2 = Positions[triArray[i * 3 + 1]];
            Vector3 v3 = Positions[triArray[i * 3 + 2]];

            Vector3 N = (Vector3.Cross(v2 - v1, v3 - v1));

            vDataArray[triArray[i * 3 + 0]].norms += N;
            vDataArray[triArray[i * 3 + 1]].norms += N;
            vDataArray[triArray[i * 3 + 2]].norms += N;
        }
        for (int i = 0; i < nodeCount; i++)
        {
            vDataArray[i].norms = vDataArray[i].norms.normalized;
        }
    }


    private void OnDrawGizmos()
    {
       
    }

    private void OnDestroy()
    {
        vertexBuffer.Dispose();
        triBuffer.Dispose();
    }

}