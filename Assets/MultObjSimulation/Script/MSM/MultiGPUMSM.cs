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
using ExporterImporter;
using UnityEngine.UIElements;

public class MultiGPUMSM : MonoBehaviour
{
    public enum MyModel
    {
        IcoSphere_low,
        Torus,
        Bunny,
        Armadillo,
        Cube
    };

    [Header("Number model")]
    public int number_object = 1;
    [Header("Random Range")]
    Vector3 rangeMin = new Vector3(-10f, 0f, 0f);
    Vector3 rangeMax = new Vector3(10f, 10f, 20f);

    [Header("3D model")]
    public MyModel model;

    [Header("Import CSV")]
    public string csv_file = "object_positions.csv";

    [HideInInspector]
    private string modelName;

    [Header("Obj Parameters")]
    public float dt = 0.005f; // have to devide by 20
    public Vector3 gravity = new Vector3(0f, -9.81f, 0f);

    public int speed = 1;

    [Header("Geometric Parameters")]
    public bool useInteriorSpring = false;

    [Header("Collision")]
    public GameObject[] collidableObject;

    [Header("GPU Paramenter")]
    public ComputeShader computeShader;
    public ComputeShader OctreeAabbCS;

    [Header("Rendering")]
    public Shader renderingShader;
    public Color matColor;

    [HideInInspector]
    private GameObject[] deformableObjectList;
    private GPUMSM[] deformableGPUMSM;
    GPUMSM gpumsmScript;

    [Header("Debug Mode")]
    public bool debugData = true;
    public bool debugLv0 = true;    
    public bool debugLv1 = true;    
    public bool debugTri = true;
    private Vector2Int[] indicies;
    private int nodeCount;
    private int triCount;
    // kernel IDs
    private int bindPositionsKernel;
    private int bindPosTrianglesKernel;
    private int findBBMinMaxKernel;
    private int implementOctreeKernel;
    private int TriIntersectionKernel;    
    private int RmCollisionResultsKernel;
    private int RemoveTriKernel;

    // Compute Buffer
    private ComputeBuffer positionsBuffer;
    private ComputeBuffer globalPositionsBuffer;
    private ComputeBuffer posTrianglesBuffer;
    private ComputeBuffer globalPosTrianglesBuffer;
    private ComputeBuffer bbMinMaxBuffer;
    private ComputeBuffer rangeObjIndexBuffer;
    private ComputeBuffer bbOctreeBuffer;
    private ComputeBuffer pairIndexL1Buffer;
    private ComputeBuffer pairIndexTriBuffer;
    private ComputeBuffer collisionResultsBuffer;
    private ComputeBuffer collisionResultTri1Buffer;
    private ComputeBuffer collisionResultTri2Buffer;
    private OctreeData[] bbOctree;
    private List<PairData> pairIndexL0 = new List<PairData>();
    private List<PairData> pairIndexL1 = new List<PairData>();
    private List<PairData> pairIndexTri = new List<PairData>();
    private readonly int octree_size = 9;
    private struct Tri
    {
        public Vector3 vertex0, vertex1, vertex2;
    }
    private Tri[] posTriangles;

    
    private int[] collisionResults;
    private int[] collisionresultsTri1;
    private int[] collisionresultsTri2;

    void Start()
    {
        
        SelectModelName();
        addDeformableObjectList();
        AddOctreePairIndex();

        FindKernelIDs();
        SetupComputeBuffer();
        SetupComputeShader();
    }

    void SelectModelName()
    {
        switch (model)
        {
            case MyModel.IcoSphere_low: modelName = "icosphere_low.1"; break;
            case MyModel.Torus: modelName = "torus.1"; break;
            case MyModel.Bunny: modelName = "bunny.1"; break;
            case MyModel.Armadillo: modelName = "Armadillo.1"; break;
            case MyModel.Cube: modelName = "33cube.1"; break;
        }
    }
    void addDeformableObjectList()
    {
        deformableObjectList = new GameObject[number_object];
        HashSet<Vector3> generatedPositions = new HashSet<Vector3>();
        deformableGPUMSM = new GPUMSM[number_object];
        indicies = new Vector2Int[number_object];
        int st_index = 0;

        List<List<string>> csvData = ExporterAndImporter.ReadCSVFile(csv_file);
        Vector3 position = new Vector3();

        for (int i = 0; i < number_object; i++)
        {
            deformableObjectList[i] = new GameObject("Deformable Object " + i);
            deformableObjectList[i].transform.SetParent(this.transform);
           
            //set position of the object 1). randomize 2).set the coord
            Vector3 randomPosition;
            do
            {
                // Generate random position within the specified range
                float x = UnityEngine.Random.Range(rangeMin.x, rangeMax.x);
                float y = UnityEngine.Random.Range(rangeMin.y, rangeMax.y);
                float z = UnityEngine.Random.Range(rangeMin.z, rangeMax.z);

                randomPosition = new Vector3(x, y, z);
            } while (generatedPositions.Contains(randomPosition));
            //deformableObjectList[i].transform.position = randomPosition;

            List<string> row = csvData[i];
            for (int j = 1; j < row.Count; j++)
            {
                float x = float.Parse(row[0]);
                float y = float.Parse(row[1]);
                float z = float.Parse(row[2]);

                position = new Vector3(x, y, z);
            }

            deformableObjectList[i].transform.position = position;
            deformableObjectList[i].transform.localScale = transform.localScale;
            deformableObjectList[i].transform.rotation = transform.rotation;

            if (deformableObjectList[i] != null)
            {
                gpumsmScript = deformableObjectList[i].AddComponent<GPUMSM>();
                gpumsmScript.SetCoefficient( dt, gravity, speed);
                gpumsmScript.SetMeshData(modelName, useInteriorSpring);
                Shader tmp = Instantiate(renderingShader);
                ComputeShader tmpCS = Instantiate(computeShader);
                Color randomColor = new Color(
                    UnityEngine.Random.value,
                    UnityEngine.Random.value,
                    UnityEngine.Random.value);
                //gpumsmScript.SetRenderer(tmp, matColor);
                gpumsmScript.SetRenderer(tmpCS, tmp, randomColor);
                gpumsmScript.SetCollidableObj(collidableObject);
                deformableGPUMSM[i] = gpumsmScript;

                deformableGPUMSM[i].StartObj();

                nodeCount = deformableGPUMSM[i].getNodeCount();
                triCount = deformableGPUMSM[i].getTriCount();

                indicies[i] = new Vector2Int(st_index, st_index + nodeCount);
                st_index += nodeCount;
            }
        }
    }

    private void AddOctreePairIndex()
    {
         for(int i = 0; i < number_object * octree_size; i++)
        {
            for (int j = i+1; j < number_object * octree_size; j++)
            {
                if(i %9==0 && j %9==0)
                {
                    pairIndexL0.Add( new PairData
                    {
                        i1 = i,
                        i2 = j
                    });

                }

                if(i %9 !=0 && j % 9 !=0)
                {
                    if (Mathf.Floor(i / 9) != Mathf.Floor(j / 9))
                    {
                        pairIndexL1.Add( new PairData
                        {
                            i1 = i,
                            i2 = j
                        });

                        //print($"{i} {j}");
                    }

                }
            }
        }
    }

    private void FindKernelIDs()
    {
        bindPositionsKernel = OctreeAabbCS.FindKernel("BindGlobalPositions");
        bindPosTrianglesKernel = OctreeAabbCS.FindKernel("BindGlobalPosTriangles");
        findBBMinMaxKernel = OctreeAabbCS.FindKernel("FindBBMinMax");
        implementOctreeKernel = OctreeAabbCS.FindKernel("ImplementOctree");
        TriIntersectionKernel = OctreeAabbCS.FindKernel("TriIntersection");
        RmCollisionResultsKernel = OctreeAabbCS.FindKernel("RemoveCollisionResults");
        RemoveTriKernel = OctreeAabbCS.FindKernel("RemoveTriKernel");
    }

    private void SetupComputeBuffer()
    {
        collisionResults = new int[number_object * octree_size];
        collisionresultsTri1 = new int[number_object * triCount];
        collisionresultsTri2 = new int[number_object * triCount];
        globalPositionsBuffer = new ComputeBuffer(nodeCount * number_object, sizeof(float) * 3);
        positionsBuffer = new ComputeBuffer(nodeCount, sizeof(float) * 3);
        posTrianglesBuffer = new ComputeBuffer(triCount, sizeof(float) * 9);
        globalPosTrianglesBuffer = new ComputeBuffer(triCount * number_object, sizeof(float) * 9);
        bbMinMaxBuffer = new ComputeBuffer(number_object, sizeof(float) * 6);
        rangeObjIndexBuffer = new ComputeBuffer(number_object, sizeof(int) * 2);
        bbOctreeBuffer = new ComputeBuffer(number_object * octree_size, sizeof(float) * 13);
        pairIndexL1Buffer = new ComputeBuffer(pairIndexL1.Count, sizeof(int) * 2);
        // pairIndexTriBuffer = new ComputeBuffer(pairIndexTri.Count, sizeof(int) * 2);
        collisionResultsBuffer = new ComputeBuffer(number_object * octree_size, sizeof(int));
        collisionResultTri1Buffer = new ComputeBuffer(number_object * triCount, sizeof(int));
        collisionResultTri2Buffer = new ComputeBuffer(number_object * triCount, sizeof(int));
    }

    private void SetupComputeShader()
    {
        bbOctree = new OctreeData[number_object * octree_size];
        posTriangles = new Tri[number_object * triCount];
        Tri[] globalPos =  new Tri[number_object * triCount];

        OctreeAabbCS.SetInt("numberObj", number_object);
        OctreeAabbCS.SetInt("nodeCount", nodeCount);
        OctreeAabbCS.SetInt("triCount", triCount);
        OctreeAabbCS.SetBool("debug", debugData);

        collisionResults.Initialize();
        collisionresultsTri1.Initialize();
        collisionresultsTri2.Initialize();
        globalPos.Initialize();

        rangeObjIndexBuffer.SetData(indicies);
        globalPosTrianglesBuffer.SetData(globalPos);
        pairIndexL1Buffer.SetData(pairIndexL1);
        collisionResultsBuffer.SetData(collisionResults);
        collisionResultTri1Buffer.SetData(collisionresultsTri1);
        collisionResultTri2Buffer.SetData(collisionresultsTri2);

        // BindGlobalPositions
        OctreeAabbCS.SetBuffer(bindPositionsKernel, "globalPositions", globalPositionsBuffer);

        // BindGlobalPosTriangles
        OctreeAabbCS.SetBuffer(bindPosTrianglesKernel, "globalPosTriangles", globalPosTrianglesBuffer);

        // findBBMinMaxKernel
        OctreeAabbCS.SetBuffer(findBBMinMaxKernel, "bbMinMax", bbMinMaxBuffer);
        OctreeAabbCS.SetBuffer(findBBMinMaxKernel, "rangeObjIndex", rangeObjIndexBuffer);
        OctreeAabbCS.SetBuffer(findBBMinMaxKernel, "globalPositions", globalPositionsBuffer);

        // ImplementOctree
        OctreeAabbCS.SetBuffer(implementOctreeKernel, "bbMinMax", bbMinMaxBuffer);
        OctreeAabbCS.SetBuffer(implementOctreeKernel, "bbOctree", bbOctreeBuffer);

        // RemoveTriKernel
        OctreeAabbCS.SetBuffer(RemoveTriKernel, "collisionResultTri1", collisionResultTri1Buffer);
        OctreeAabbCS.SetBuffer(RemoveTriKernel, "collisionResultTri2", collisionResultTri2Buffer);

        // RemoveTriKernel
        OctreeAabbCS.SetBuffer(RmCollisionResultsKernel, "collisionResults", collisionResultsBuffer);

        // TriIntersection
        OctreeAabbCS.SetBuffer(TriIntersectionKernel, "globalPosTriangles", globalPosTrianglesBuffer); 
        OctreeAabbCS.SetBuffer(TriIntersectionKernel, "collisionResultTri1", collisionResultTri1Buffer); 
        OctreeAabbCS.SetBuffer(TriIntersectionKernel, "collisionResultTri2", collisionResultTri2Buffer); 
        
        OctreeAabbCS.SetBuffer(TriIntersectionKernel, "pairIndexL1", pairIndexL1Buffer);
        OctreeAabbCS.SetBuffer(TriIntersectionKernel, "bbMinMax", bbMinMaxBuffer);
        OctreeAabbCS.SetBuffer(TriIntersectionKernel, "bbOctree", bbOctreeBuffer);
        OctreeAabbCS.SetBuffer(TriIntersectionKernel, "collisionResults", collisionResultsBuffer);
      
    }


    private void Update()
    {
        for (int i = 0; i < number_object; i++)
        {
            OctreeAabbCS.SetInt("objectIndex", i);
            deformableGPUMSM[i].UpdateObj();

            //if (deformableGPUMSM[i].GetPositionBuffer() != null) positionsBuffer.Release();
            positionsBuffer = deformableGPUMSM[i].GetPositionBuffer();
            posTrianglesBuffer = deformableGPUMSM[i].GetPosTrianglesBuffer();

            // ??
            OctreeAabbCS.SetBuffer(bindPositionsKernel, "positions", positionsBuffer);
            
            // ??
            OctreeAabbCS.SetBuffer(bindPosTrianglesKernel, "posTriangles", posTrianglesBuffer);


            int numGroups_Pos = Mathf.CeilToInt(nodeCount / 1024f);
            OctreeAabbCS.Dispatch(bindPositionsKernel, numGroups_Pos, 1, 1);

            int numGroups_Tri = Mathf.CeilToInt(triCount * number_object / 1024f);
            OctreeAabbCS.Dispatch(bindPosTrianglesKernel, numGroups_Tri, numGroups_Tri, 1);
        }

        DispatchComputeShader();
        GetDataToCPU();
    }

    private void DispatchComputeShader()
    {
        OctreeAabbCS.Dispatch(findBBMinMaxKernel, Mathf.CeilToInt(number_object / 1024f), 1, 1);
        OctreeAabbCS.Dispatch(implementOctreeKernel, Mathf.CeilToInt(number_object / 1024f), 1, 1);
        OctreeAabbCS.Dispatch(RemoveTriKernel, Mathf.CeilToInt(triCount * number_object / 1024f), 1, 1);
        OctreeAabbCS.Dispatch(RmCollisionResultsKernel, Mathf.CeilToInt(9 * number_object / 1024f), 1, 1);
        
        int numGroups_Tri =  Mathf.CeilToInt(triCount * number_object / 32);
        OctreeAabbCS.Dispatch(TriIntersectionKernel, numGroups_Tri, numGroups_Tri, 1);             
    }

    private void GetDataToCPU()
    {
        if (debugData)
        {
            bbOctreeBuffer.GetData(bbOctree);
            collisionResultsBuffer.GetData(collisionResults);

        } 

        if(debugTri)
        {
            collisionResultTri1Buffer.GetData(collisionresultsTri1);
            collisionResultTri2Buffer.GetData(collisionresultsTri2);
            globalPosTrianglesBuffer.GetData(posTriangles);
        }
    }

    private void OnGUI()
    {
        int w = Screen.width, h = Screen.height;
        GUIStyle style = new GUIStyle();
        Rect rect = new Rect(20, 40, w, h * 2 / 100);
        style.alignment = TextAnchor.UpperLeft;
        style.fontSize = h * 2 / 50;
        style.normal.textColor = Color.yellow;


        string text = string.Format("num. Obj :: " + number_object);
        GUI.Label(rect, text, style);

    }


    private void OnDrawGizmos()
    {
       if (bbOctree != null && debugData)
        {
            
            for (int i = 0; i < collisionResults.Length; i++)
            {
                 if(collisionResults[i] == 1)
                 {
                    if(debugLv0)
                    {
                        Gizmos.color = Color.red;
                        for (int p = 0; p < pairIndexL0.Count; p++)
                        {
                            if (i == pairIndexL0[p].i1 )
                            {
                                Gizmos.DrawWireCube((bbOctree[i].max + bbOctree[i].min)/2, bbOctree[i].max - bbOctree[i].min);
                            }

                            if (i == pairIndexL0[p].i2)
                            {
                                Gizmos.DrawWireCube((bbOctree[i].max + bbOctree[i].min)/2, bbOctree[i].max - bbOctree[i].min);
                            }

                        }
                    }
                    
                    if(debugLv1)
                    {
                        Gizmos.color = Color.green;
                        for (int p = 0; p < pairIndexL1.Count; p++)
                        {
                            if (i == pairIndexL1[p].i1 )
                            {
                                Gizmos.DrawWireCube((bbOctree[i].max + bbOctree[i].min)/2, bbOctree[i].max - bbOctree[i].min);
                            }

                            if (i == pairIndexL1[p].i2)
                            {
                                Gizmos.DrawWireCube((bbOctree[i].max + bbOctree[i].min)/2, bbOctree[i].max - bbOctree[i].min);
                            }

                        }
                    }
                    
                 }
                
            } 
        }

        if(posTriangles != null && debugTri)
        {
            for (int c = 0; c < collisionresultsTri1.Length; c++)
            {
                if (collisionresultsTri1[c] == 1)  DrawTriangle(posTriangles[c], Color.blue);                
            }

            for (int c = 0; c < collisionresultsTri2.Length; c++)
            {
                if (collisionresultsTri2[c] == 1)  DrawTriangle(posTriangles[c], Color.blue);
            }
           
        }


        
        // if (bbOctree != null && debugData)
        // {
        //     for (int i = 0; i < bbOctree.Length; i++)
        //     {
        //         if(i%9==0)
        //         {
        //            Gizmos.color = Color.red;
        //         }
        //         if(i%9!=0)
        //         {
        //         Gizmos.color = Color.green;
        //         Gizmos.DrawWireCube(bbOctree[i].center, bbOctree[i].max - bbOctree[i].min);
        //         }
                
        //     } 
        // }


    }

    private void DrawTriangle(Tri triangle, Color color)
    {
        Gizmos.color = color;
        Gizmos.DrawLine(triangle.vertex0, triangle.vertex1);
        Gizmos.DrawLine(triangle.vertex1, triangle.vertex2);
        Gizmos.DrawLine(triangle.vertex2, triangle.vertex0);
    }


    private void OnDestroy()
    {
        if (enabled)
        {
            positionsBuffer.Release();
            globalPositionsBuffer.Release();
            posTrianglesBuffer.Release();
            globalPosTrianglesBuffer.Release();
            bbMinMaxBuffer.Release();
            rangeObjIndexBuffer.Release();
            
            pairIndexL1Buffer.Release();
            collisionResultsBuffer.Release();
            collisionResultTri1Buffer.Release();
            collisionResultTri2Buffer.Release();
        }
    }
}