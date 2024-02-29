using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;
using System.Text;
using System;
using System.Linq;
using UnityEngine.Rendering;
using PBD;

public class GPUPBD : MonoBehaviour
{

    
    //property
    private float invMass = 1.0f;
    private float dt = 0.01f; // have to devide by 20
    private Vector3 gravity = new Vector3(0f, -9.81f, 0f);
    private int iteration = 1;

    private float stretchStiffness = 1.0f;
    private float compressStiffness = 1.0f;
    private float bendingStiffness = 1.0f;
    private float volumeStiffness = 1.0f;
    private float convergence_factor = 1.5f;
    private GameObject[] collidableObjects;
    private GameObject floorObj;
    //private

    private int nodeCount;
    private int springCount;
    private int triCount; // size of triangle
    private int tetCount;
    private int bendingCount;


    private float totalVolume;
    private string modelName;
    private Vector3[] Positions;
    private Vector3[] ProjectPositions;
    private Vector3[] WorldPositions;
    private Vector3[] Velocities;
    private Vector3[] Forces;
    private Vector3[] Normals;
    private List<Spring> distanceConstraints = new List<Spring>();
    private List<Triangle> triangles = new List<Triangle>();
    private List<Tetrahedron> tetrahedrons = new List<Tetrahedron>();
    private List<Bending> bendingConstraints = new List<Bending>();

    private int[] triArray;
    private vertData[] vDataArray;

    private Vector3[] DeltaPos;
    private int[] deltaCounter;


    private bool[] collidedNodes;

    //for render
    private ComputeBuffer vertsBuff;
    private ComputeBuffer triBuffer;

    private ComputeBuffer positionsBuffer;
    private ComputeBuffer projectedPositionsBuffer;
    private ComputeBuffer velocitiesBuffer;
    private ComputeBuffer triangleBuffer;
    private ComputeBuffer triangleIndicesBuffer;
    private ComputeBuffer deltaPositionsBuffer;
    private ComputeBuffer deltaPositionsUIntBuffer;
    private ComputeBuffer deltaCounterBuffer;
    private ComputeBuffer objVolumeBuffer;
    private ComputeBuffer distanceConstraintsBuffer;
    private ComputeBuffer bendingConstraintsBuffer;
    private ComputeBuffer tetVolConstraintsBuffer;

    private int applyExplicitEulerKernel;
    private int floorCollisionKernel;
    private int satisfyDistanceConstraintKernel;
    private int satisfyBendingConstraintKernel;
    private int satisfyTetVolConstraintKernel;
    private int averageConstraintDeltasKernel;
    private int updatePositionsKernel;
    private int computeObjVolumeKernel;   // for compute object's volume
    private int computeVerticesNormal; // for rendering purpose 


    private ComputeShader computeShaderobj;
    private Shader renderingShader;
    private Color matColor;
    private Material material;


   

    public void SetCoefficient(float Inv_Mass, float Dt, Vector3 Gravity, int Iteration,
        float StretchStiffness, float CompressStiffnes, float BendingStiffness, float VolumeStiffness)
    {
        invMass = Inv_Mass;
        dt = Dt;
        gravity = Gravity;
        iteration = Iteration;
        stretchStiffness = StretchStiffness;
        compressStiffness = CompressStiffnes;
        bendingStiffness = BendingStiffness;
        volumeStiffness = VolumeStiffness;
    }
    public void SetMeshData(Vector3[] NodePositions, List<Triangle> MeshTriangle,
        int[] TriArray, List<Spring> Springs, List<Bending> Bendings, List<Tetrahedron> Tetrahedrons)
    {
        Positions = NodePositions;
        triangles = MeshTriangle;
        triArray = TriArray;
        distanceConstraints = Springs;
        bendingConstraints = Bendings;
        tetrahedrons = Tetrahedrons;
    }

    public void SetMeshData(string TetGenModel)
    {
        modelName = TetGenModel;
    }

    public void SetCollidableObj(GameObject[] CollidableObjects)
    {
        collidableObjects = CollidableObjects;
    }
    public void SetRenderer(ComputeShader computeShader,Shader RenderingShader, Color MatColor)
    {
        computeShaderobj = computeShader;
        renderingShader = RenderingShader;
        matColor = MatColor;
    }

    private void transformMesh()
    {
        string filePath = Application.dataPath + "/MultObjSimulation/TetModel/";
        LoadTetModel.LoadData(filePath + modelName, this.gameObject);

        Positions = LoadTetModel.positions.ToArray();
        triangles = LoadTetModel.triangles;
        distanceConstraints = LoadTetModel.springs;
        triArray = LoadTetModel.triangleArr.ToArray();
        //if (useTetVolumeConstraint)
        tetrahedrons = LoadTetModel.tetrahedrons;
        bendingConstraints = LoadTetModel.bendings;

        nodeCount = Positions.Length;
        springCount = distanceConstraints.Count;
        triCount = triangles.Count; //
        tetCount = tetrahedrons.Count;
        bendingCount = bendingConstraints.Count;

        WorldPositions = new Vector3[nodeCount];
        ProjectPositions = new Vector3[nodeCount];
        Velocities = new Vector3[nodeCount];
        Forces = new Vector3[nodeCount];
        DeltaPos = new Vector3[nodeCount];
        deltaCounter = new int[nodeCount];
        DeltaPos.Initialize();
        deltaCounter.Initialize();

        ProjectPositions = LoadTetModel.positions.ToArray();
        WorldPositions.Initialize();
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
        triBuffer = new ComputeBuffer(triArray.Length,
            triBuffStride, ComputeBufferType.Default);


        int vertsBuffstride = 8 * sizeof(float);
        vertsBuff = new ComputeBuffer(vDataArray.Length,
            vertsBuffstride, ComputeBufferType.Default);
        LoadTetModel.ClearData();

        print("node count: " + nodeCount);
        print("stretch constraint: " + springCount);
        print("bending constraint: " + bendingCount);
        print("volume constraint: " + tetCount);

    }

    private void setupShader()
    {
        material.SetBuffer(Shader.PropertyToID("vertsBuff"), vertsBuff);
        material.SetBuffer(Shader.PropertyToID("triBuff"), triBuffer);
    }
    private void setBuffData()
    {

        vertsBuff.SetData(vDataArray);
        triBuffer.SetData(triArray);
        //Quaternion rotate = new Quaternion(0, 0, 0, 0);
        //transform.Rotate(rotate.eulerAngles);
        //transform.
        Vector3 translation = transform.position;
        Vector3 scale = this.transform.localScale;
        Quaternion rotationeuler = transform.rotation;
        Matrix4x4 trs = Matrix4x4.TRS(translation, rotationeuler, scale);
        material.SetMatrix("TRSMatrix", trs);
        material.SetMatrix("invTRSMatrix", trs.inverse);
    }
    private void setupComputeBuffer()
    {
        positionsBuffer = new ComputeBuffer(nodeCount, sizeof(float) * 3);
        positionsBuffer.SetData(Positions);

        velocitiesBuffer = new ComputeBuffer(nodeCount, sizeof(float) * 3);
        velocitiesBuffer.SetData(Velocities);

        projectedPositionsBuffer = new ComputeBuffer(nodeCount, sizeof(float) * 3);
        projectedPositionsBuffer.SetData(Positions);

        UInt3Struct[] deltaPosUintArray = new UInt3Struct[nodeCount];
        deltaPosUintArray.Initialize();
        Vector3[] deltaPositionArray = new Vector3[nodeCount];
        deltaPositionArray.Initialize();

        int[] deltaCounterArray = new int[nodeCount];
        deltaCounterArray.Initialize();


        deltaPositionsBuffer = new ComputeBuffer(nodeCount, sizeof(float) * 3);
        deltaPositionsBuffer.SetData(deltaPositionArray);

        deltaPositionsUIntBuffer = new ComputeBuffer(nodeCount, sizeof(uint) * 3);
        deltaPositionsUIntBuffer.SetData(deltaPosUintArray);

        deltaCounterBuffer = new ComputeBuffer(nodeCount, sizeof(int));
        deltaCounterBuffer.SetData(deltaCounterArray);




        List<MTriangle> initTriangle = new List<MTriangle>();  //list of triangle cooresponding to node 
        List<int> initTrianglePtr = new List<int>(); //contain a group of affectd triangle to node
        initTrianglePtr.Add(0);

        for (int i = 0; i < nodeCount; i++)
        {
            foreach (Triangle tri in triangles)
            {
                if (tri.vertices[0] == i || tri.vertices[1] == i || tri.vertices[2] == i)
                {
                    MTriangle tmpTri = new MTriangle();
                    tmpTri.v0 = tri.vertices[0];
                    tmpTri.v1 = tri.vertices[1];
                    tmpTri.v2 = tri.vertices[2];
                    initTriangle.Add(tmpTri);
                }
            }
            initTrianglePtr.Add(initTriangle.Count);
        }

        print(initTrianglePtr.Count);

        triangleBuffer = new ComputeBuffer(initTriangle.Count, (sizeof(int) * 3));
        triangleBuffer.SetData(initTriangle.ToArray());

        triangleIndicesBuffer = new ComputeBuffer(initTrianglePtr.Count, sizeof(int));
        triangleIndicesBuffer.SetData(initTrianglePtr.ToArray());


        distanceConstraintsBuffer = new ComputeBuffer(springCount, sizeof(float) + sizeof(int) * 2);
        distanceConstraintsBuffer.SetData(distanceConstraints.ToArray());

        bendingConstraintsBuffer = new ComputeBuffer(bendingCount, sizeof(float) + sizeof(int) * 4);
        bendingConstraintsBuffer.SetData(bendingConstraints.ToArray());

        tetVolConstraintsBuffer = new ComputeBuffer(tetCount, sizeof(float) + sizeof(int) * 4);
        tetVolConstraintsBuffer.SetData(tetrahedrons.ToArray());

        uint[] initUint = new uint[1];
        initUint.Initialize();
        objVolumeBuffer = new ComputeBuffer(1, sizeof(uint));
        objVolumeBuffer.SetData(initUint);



    }
    private void setupKernel()
    {
        applyExplicitEulerKernel = computeShaderobj.FindKernel("applyExplicitEulerKernel");

        floorCollisionKernel = computeShaderobj.FindKernel("floorCollisionKernel");
        //for solving all constraint at once
        //satisfyPointConstraintsKernel = computeShaderobj.FindKernel("projectConstraintDeltasKernel");
        //satisfySphereCollisionsKernel = computeShaderobj.FindKernel("projectConstraintDeltasKernel");
        //satisfyCubeCollisionsKernel = computeShaderobj.FindKernel("projectConstraintDeltasKernel");
        //for solving constrint one-by-one
        satisfyDistanceConstraintKernel = computeShaderobj.FindKernel("satisfyDistanceConstraintKernel");
        averageConstraintDeltasKernel = computeShaderobj.FindKernel("averageConstraintDeltasKernel");

        satisfyBendingConstraintKernel = computeShaderobj.FindKernel("satisfyBendingConstraintKernel");
        satisfyTetVolConstraintKernel = computeShaderobj.FindKernel("satisfyTetVolConstraintKernel");

        //update position
        updatePositionsKernel = computeShaderobj.FindKernel("updatePositionsKernel");
        //object volume
        computeObjVolumeKernel = computeShaderobj.FindKernel("computeObjVolumeKernel");
        //for rendering
        computeVerticesNormal = computeShaderobj.FindKernel("computeVerticesNormal");

    }
    private void setupComputeShader()
    {
        //send uniform data for kernels in compute shader
        computeShaderobj.SetInt("nodeCount", nodeCount);
        computeShaderobj.SetInt("springCount", springCount);
        computeShaderobj.SetInt("triCount", triCount);
        computeShaderobj.SetInt("tetCount", tetCount);
        computeShaderobj.SetInt("bendingCount", bendingCount);

        computeShaderobj.SetFloat("dt", dt);
        computeShaderobj.SetFloat("invMass", invMass);
        computeShaderobj.SetFloat("stretchStiffness", stretchStiffness);
        computeShaderobj.SetFloat("compressStiffness", compressStiffness);
        computeShaderobj.SetFloat("bendingStiffness", bendingStiffness);
        computeShaderobj.SetFloat("tetVolStiffness", volumeStiffness);
        computeShaderobj.SetFloat("convergence_factor", convergence_factor);

        computeShaderobj.SetVector("gravity", gravity);

        // bind buffer data to each kernel

        //Kernel #1 add force & apply euler
        computeShaderobj.SetBuffer(applyExplicitEulerKernel, "Velocities", velocitiesBuffer);
        computeShaderobj.SetBuffer(applyExplicitEulerKernel, "Positions", positionsBuffer);
        computeShaderobj.SetBuffer(applyExplicitEulerKernel, "ProjectedPositions", projectedPositionsBuffer);

        //Kernel #2
        computeShaderobj.SetBuffer(satisfyDistanceConstraintKernel, "deltaPos", deltaPositionsBuffer);            //for find the correct project position
        computeShaderobj.SetBuffer(satisfyDistanceConstraintKernel, "deltaPosAsInt", deltaPositionsUIntBuffer);   //for find the correct project position
        computeShaderobj.SetBuffer(satisfyDistanceConstraintKernel, "deltaCount", deltaCounterBuffer);            //for find the correct project position
        computeShaderobj.SetBuffer(satisfyDistanceConstraintKernel, "ProjectedPositions", projectedPositionsBuffer);
        computeShaderobj.SetBuffer(satisfyDistanceConstraintKernel, "distanceConstraints", distanceConstraintsBuffer);
        computeShaderobj.SetBuffer(satisfyDistanceConstraintKernel, "Positions", positionsBuffer);

        computeShaderobj.SetBuffer(satisfyBendingConstraintKernel, "deltaPos", deltaPositionsBuffer);            //for find the correct project position
        computeShaderobj.SetBuffer(satisfyBendingConstraintKernel, "deltaPosAsInt", deltaPositionsUIntBuffer);   //for find the correct project position
        computeShaderobj.SetBuffer(satisfyBendingConstraintKernel, "deltaCount", deltaCounterBuffer);            //for find the correct project position
        computeShaderobj.SetBuffer(satisfyBendingConstraintKernel, "ProjectedPositions", projectedPositionsBuffer);
        computeShaderobj.SetBuffer(satisfyBendingConstraintKernel, "bendingConstraints", bendingConstraintsBuffer);
        computeShaderobj.SetBuffer(satisfyBendingConstraintKernel, "Velocities", velocitiesBuffer);
        computeShaderobj.SetBuffer(satisfyBendingConstraintKernel, "Positions", positionsBuffer);

        computeShaderobj.SetBuffer(satisfyTetVolConstraintKernel, "deltaPos", deltaPositionsBuffer);            //for find the correct project position
        computeShaderobj.SetBuffer(satisfyTetVolConstraintKernel, "deltaPosAsInt", deltaPositionsUIntBuffer);   //for find the correct project position
        computeShaderobj.SetBuffer(satisfyTetVolConstraintKernel, "deltaCount", deltaCounterBuffer);            //for find the correct project position
        computeShaderobj.SetBuffer(satisfyTetVolConstraintKernel, "ProjectedPositions", projectedPositionsBuffer);
        computeShaderobj.SetBuffer(satisfyTetVolConstraintKernel, "tetVolumeConstraints", tetVolConstraintsBuffer);
        computeShaderobj.SetBuffer(satisfyTetVolConstraintKernel, "Velocities", velocitiesBuffer);
        computeShaderobj.SetBuffer(satisfyTetVolConstraintKernel, "Positions", positionsBuffer);

        computeShaderobj.SetBuffer(averageConstraintDeltasKernel, "deltaPos", deltaPositionsBuffer);            //for find the correct project position
        computeShaderobj.SetBuffer(averageConstraintDeltasKernel, "deltaPosAsInt", deltaPositionsUIntBuffer);   //for find the correct project position
        computeShaderobj.SetBuffer(averageConstraintDeltasKernel, "deltaCount", deltaCounterBuffer);            //for find the correct project position
        computeShaderobj.SetBuffer(averageConstraintDeltasKernel, "ProjectedPositions", projectedPositionsBuffer);
        computeShaderobj.SetBuffer(averageConstraintDeltasKernel, "distanceConstraints", distanceConstraintsBuffer);
        computeShaderobj.SetBuffer(averageConstraintDeltasKernel, "Velocities", velocitiesBuffer);
        computeShaderobj.SetBuffer(averageConstraintDeltasKernel, "Positions", positionsBuffer);

        computeShaderobj.SetBuffer(floorCollisionKernel, "Velocities", velocitiesBuffer);
        computeShaderobj.SetBuffer(floorCollisionKernel, "Positions", positionsBuffer);
        computeShaderobj.SetBuffer(floorCollisionKernel, "ProjectedPositions", projectedPositionsBuffer);
        //Kernel  update position
        //computeShaderobj.SetBuffer(applyExplicitEulerKernel, "Velocities", velocitiesBuffer);
        computeShaderobj.SetBuffer(updatePositionsKernel, "Velocities", velocitiesBuffer);
        computeShaderobj.SetBuffer(updatePositionsKernel, "Positions", positionsBuffer);
        computeShaderobj.SetBuffer(updatePositionsKernel, "ProjectedPositions", projectedPositionsBuffer);
        computeShaderobj.SetBuffer(updatePositionsKernel, "vertsBuff", vertsBuff); //passing to rendering

        computeShaderobj.SetBuffer(computeObjVolumeKernel, "objVolume", objVolumeBuffer);
        computeShaderobj.SetBuffer(computeObjVolumeKernel, "Positions", positionsBuffer);
        computeShaderobj.SetBuffer(computeObjVolumeKernel, "tetVolumeConstraints", tetVolConstraintsBuffer);

        //kernel compute vertices normal
        computeShaderobj.SetBuffer(computeVerticesNormal, "Positions", positionsBuffer);
        computeShaderobj.SetBuffer(computeVerticesNormal, "Triangles", triangleBuffer);
        computeShaderobj.SetBuffer(computeVerticesNormal, "TrianglePtr", triangleIndicesBuffer);
        computeShaderobj.SetBuffer(computeVerticesNormal, "vertsBuff", vertsBuff); //passing to rendering
        computeShaderobj.SetBuffer(computeVerticesNormal, "objVolume", objVolumeBuffer);
    }
    private float computeTetraVolume(Vector3 i1, Vector3 i2, Vector3 i3, Vector3 i4)
    {
        float volume = 0.0f;

        volume = 1.0f / 6.0f
            * (i3.x * i2.y * i1.z - i4.x * i2.y * i1.z - i2.x * i3.y * i1.z
            + i4.x * i3.y * i1.z + i2.x * i4.y * i1.z - i3.x * i4.y * i1.z
            - i3.x * i1.y * i2.z + i4.x * i1.y * i2.z + i1.x * i3.y * i2.z
            - i4.x * i3.y * i2.z - i1.x * i4.y * i2.z + i3.x * i4.y * i2.z
            + i2.x * i1.y * i3.z - i4.x * i1.y * i3.z - i1.x * i2.y * i3.z
            + i4.x * i2.y * i3.z + i1.x * i4.y * i3.z - i2.x * i4.y * i3.z
            - i2.x * i1.y * i4.z + i3.x * i1.y * i4.z + i1.x * i2.y * i4.z
            - i3.x * i2.y * i4.z - i1.x * i3.y * i4.z + i2.x * i3.y * i4.z);

        return volume;
    }
    float computeObjectVolume()
    {
        //made by sum of all tetra
        float volume = 0.0f;
        foreach (Tetrahedron tet in tetrahedrons)
        {
            Vector3 p0 = Positions[tet.i1];
            Vector3 p1 = Positions[tet.i2];
            Vector3 p2 = Positions[tet.i3];
            Vector3 p3 = Positions[tet.i4];
            volume += computeTetraVolume(p0, p1, p2, p3);
        }
        return volume;
    }
    void Start()
    {
        //print(gameObject.name + " :: " + dt);
        transformMesh();

        if(collidableObjects != null) floorObj = collidableObjects[0];
        material = new Material(renderingShader);
        material.color = matColor;

        setupShader();
        setBuffData(); 
        setupComputeBuffer();
        setupKernel();
        setupComputeShader();
        totalVolume = computeObjectVolume();

    }

    // Update is called once per frame
    void dispatchComputeShader()
    {
        ////update uniform data and GPU buffer here
        ////PBD algorithm
        computeShaderobj.Dispatch(applyExplicitEulerKernel, (int)Mathf.Ceil(nodeCount / 1024.0f), 1, 1);
        ////damp velocity() here
        for (int i = 0; i < iteration; i++)
        {
            //solving constraint using avaerage jacobi style
            //convergence rate slower that Gauss–Seidel method implement on CPU method
            computeShaderobj.Dispatch(satisfyDistanceConstraintKernel, (int)Mathf.Ceil(springCount / 1024.0f), 1, 1);
            computeShaderobj.Dispatch(satisfyBendingConstraintKernel, (int)Mathf.Ceil(bendingCount / 1024.0f), 1, 1);
            computeShaderobj.Dispatch(satisfyTetVolConstraintKernel, (int)Mathf.Ceil(tetCount / 1024.0f), 1, 1);
            computeShaderobj.Dispatch(averageConstraintDeltasKernel, (int)Mathf.Ceil(nodeCount / 1024.0f), 1, 1);

            computeShaderobj.SetFloat("floorCoordY", floorObj.transform.position.y);
            computeShaderobj.Dispatch(floorCollisionKernel, (int)Mathf.Ceil(nodeCount / 1024.0f), 1, 1);
        }
        computeShaderobj.Dispatch(updatePositionsKernel, (int)Mathf.Ceil(nodeCount / 1024.0f), 1, 1);

        //compute normal for rendering
        computeShaderobj.Dispatch(computeVerticesNormal, (int)Mathf.Ceil(nodeCount / 1024.0f), 1, 1);
    }
    void renderObject()
    {
        Bounds bounds = new Bounds(Vector3.zero, Vector3.one * 10000);
        material.SetPass(0);
        Graphics.DrawProcedural(material, bounds, MeshTopology.Triangles, triArray.Length,
            1, null, null, ShadowCastingMode.On, true, gameObject.layer);

    }
    void Update()
    {

        dispatchComputeShader();
        renderObject();
    }

    private void OnDestroy()
    {

        if (this.enabled)
        {
            vertsBuff.Dispose();
            triBuffer.Dispose();

            triangleBuffer.Dispose();
            triangleIndicesBuffer.Dispose();

            positionsBuffer.Dispose();
            velocitiesBuffer.Dispose();
            projectedPositionsBuffer.Dispose();
            deltaPositionsBuffer.Dispose();
            deltaPositionsUIntBuffer.Dispose();
            distanceConstraintsBuffer.Dispose();
            bendingConstraintsBuffer.Dispose();
            tetVolConstraintsBuffer.Dispose();
            objVolumeBuffer.Dispose();
            deltaCounterBuffer.Dispose();
        }

    }

    
}
