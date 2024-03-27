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
using UnityEngine.Video;

public class MultiCPUMSM : MonoBehaviour
{
    public enum MyModel
    {
        IcoSphere_low,
        Torus,
        Bunny,
        Armadillo,
        Cube
    };

    [Header("Deformable model")]
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

    [Header("Rendering")]
    public Shader renderingShader;
    public Color matColor;

    [Header("Debug Mode")]
    public bool debugModeLv0 = true; 
    public bool debugModeLv1 = true; 
    public bool debugModeTriangle = true; 

    [HideInInspector]
    private GameObject[] deformableObjectList;
    private CPUMSM[] deformableCPUMSM;
    private List<PairData> pairIndexL0 = new List<PairData>();
    private List<PairData> pairIndexL1 = new List<PairData>();
    private readonly int octree_size = 9;
    
    private OctreeData[] custOctree;
    // collidable pair
    private readonly List<int> collidablePairIndexL0 = new List<int>(); 
    private readonly List<int> collidablePairIndexL1 = new List<int>(); 
    private readonly List<Vector2Int> collidableTriIndex = new List<Vector2Int>();

    private int triCount;

    public class Tris
    {
        public Vector3 vertex0, vertex1, vertex2; // �ﰢ���� ������
        public Vector3 p_vertex0, p_vertex1, p_vertex2; // �ﰢ���� ���� ��ġ
        public Vector3 vel0, vel1, vel2;

        public Vector3 gravity = new Vector3(0.0f, -9.8f, 0.0f);
        public float deltaTime = 0.01f;

        public void setZeroGravity()
        {
            this.gravity = Vector3.zero;
        }

        public void setInverseGravity()
        {
            this.gravity *= -1.0f;
        }

        public Vector3 getAverageVelocity()
        {
            return (this.vel0 + this.vel1 + this.vel2) / 3.0f;
        }
    }

     public class Line
    {
        public Vector3 p0, p1; // �ﰢ���� ������

        public Vector3 direction
        {
            get { return (p1 - p0).normalized; }
        }

        public Vector3 origin
        {
            get { return p0; }
        }
    }

    private List<Tris> posTriangle;

    Vector3 hitPoint = new Vector3();
    float separationDistance = 0.05f;

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

    void Awake()
    {
        SelectModelName();
        addDeformableObjectList();
        AddOctreePairIndex();
    }

    void addDeformableObjectList()
    {
        deformableObjectList = new GameObject[number_object];
        deformableCPUMSM = new CPUMSM[number_object];
        custOctree = new OctreeData[number_object * octree_size];

        List<List<string>> csvData = ExporterAndImporter.ReadCSVFile(csv_file);
        Vector3 position = new Vector3();

        for (int i = 0; i < number_object; i++)
        {
            deformableObjectList[i] = new GameObject("Deformable Object " + i);
            deformableObjectList[i].transform.SetParent(transform);

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
                CPUMSM cpumsmScript = deformableObjectList[i].AddComponent<CPUMSM>();

                cpumsmScript.SetCoefficient(dt, gravity, speed);
                cpumsmScript.SetMeshData(modelName, useInteriorSpring);

                Shader tmp = Instantiate(renderingShader);
                Color randomColor = new Color(
                    UnityEngine.Random.value,
                    UnityEngine.Random.value,
                    UnityEngine.Random.value);

                //cpumsmScript.SetRenderer(tmp, matColor);
                cpumsmScript.SetRenderer(tmp, randomColor);
                cpumsmScript.SetCollidableObj(collidableObject);

                deformableCPUMSM[i] = cpumsmScript;
                deformableCPUMSM[i].StartObj();

                triCount = deformableCPUMSM[i].getTriCount();
            }

            posTriangle = new List<Tris>();
        }
    }

    private void AddOctreePairIndex()
    {
        for(int i = 0; i < number_object * octree_size; i++)
        {
            for (int j = i+1; j < number_object * octree_size; j++)
            {
                // if(i %9==0 && j %9==0)
                // {
                //     pairIndexL0.Add( new PairData
                //     {
                //         i1 = i,
                //         i2 = j
                //     });

                // }

                if(i %9 !=0 && j % 9 !=0)
                {
                    if (Mathf.Floor(i / 9) != Mathf.Floor(j / 9))
                    {
                        //print($" {i} {j}");
                        pairIndexL1.Add( new PairData
                        {
                            i1 = i,
                            i2 = j
                        });
                    }

                }
            }
        }
    }

    
    private void Update()
    {
        
        posTriangle.Clear();
        for (int i = 0; i < number_object; i++)
        {
            ImplementOctree(i);
            AddTriangles(i);
        }
        
        
        CheckTriIntersection();
    }

    private void ImplementOctree(int i) 
    {
        OctreeData[] custBB = new OctreeData[number_object];
        {
            deformableCPUMSM[i].UpdateObj();
            Vector3[] position = deformableCPUMSM[i].GetPosition();

            CustMesh.Getvertices(position);
            custBB[i].min = CustMesh.minPos;
            custBB[i].max = CustMesh.maxPos;
            custBB[i].center = (custBB[i].max + custBB[i].min) / 2;
            custBB[i].size = custBB[i].max - custBB[i].min;

            deformableCPUMSM[i].SetBoundingBox(custBB[i].min, custBB[i].max);

            // Lv0, Initialize Lv0
            custOctree[i * octree_size].center = custBB[i].center; // center Lv0
            custOctree[i * octree_size].size = custBB[i].size; // size Lv0
            custOctree[i * octree_size].min = custBB[i].min; // min value Lv0
            custOctree[i * octree_size].max = custBB[i].max; // max value Lv0

            Vector3 centerOct = custOctree[i * octree_size].center;
            Vector3 sizeOct = custOctree[i * octree_size].size;

             // Lv1, Split to 8 children [0-7] 
            custOctree[i * octree_size + 1].center.x = centerOct.x - (sizeOct.x / 4);
            custOctree[i * octree_size + 1].center.y = centerOct.y + (sizeOct.y / 4);
            custOctree[i * octree_size + 1].center.z = centerOct.z - (sizeOct.z / 4);

            custOctree[i * octree_size + 2].center.x = centerOct.x + (sizeOct.x / 4);
            custOctree[i * octree_size + 2].center.y = centerOct.y + (sizeOct.y / 4);
            custOctree[i * octree_size + 2].center.z = centerOct.z - (sizeOct.z / 4);

            custOctree[i * octree_size + 3].center.x = centerOct.x - (sizeOct.x / 4);
            custOctree[i * octree_size + 3].center.y = centerOct.y - (sizeOct.y / 4);
            custOctree[i * octree_size + 3].center.z = centerOct.z - (sizeOct.z / 4);

            custOctree[i * octree_size + 4].center.x = centerOct.x + (sizeOct.x / 4);
            custOctree[i * octree_size + 4].center.y = centerOct.y - (sizeOct.y / 4);
            custOctree[i * octree_size + 4].center.z = centerOct.z - (sizeOct.z / 4);

            custOctree[i * octree_size + 5].center.x = centerOct.x - (sizeOct.x / 4);
            custOctree[i * octree_size + 5].center.y = centerOct.y + (sizeOct.y / 4);
            custOctree[i * octree_size + 5].center.z = centerOct.z + (sizeOct.z / 4);

            custOctree[i * octree_size + 6].center.x = centerOct.x + (sizeOct.x / 4);
            custOctree[i * octree_size + 6].center.y = centerOct.y + (sizeOct.y / 4);
            custOctree[i * octree_size + 6].center.z = centerOct.z + (sizeOct.z / 4);

            custOctree[i * octree_size + 7].center.x = centerOct.x - (sizeOct.x / 4);
            custOctree[i * octree_size + 7].center.y = centerOct.y - (sizeOct.y / 4);
            custOctree[i * octree_size + 7].center.z = centerOct.z + (sizeOct.z / 4);

            custOctree[i * octree_size + 8].center.x = centerOct.x + (sizeOct.x / 4);
            custOctree[i * octree_size + 8].center.y = centerOct.y - (sizeOct.y / 4);
            custOctree[i * octree_size + 8].center.z = centerOct.z + (sizeOct.z / 4);

            for (int j = 1; j <= 8; j++)
            {
                OctreeData oct = custOctree[i * octree_size + j];
                // find min and max value of Lv1
                custOctree[i * octree_size + j].min = oct.Minimum(oct.center, custBB[i].size / 4);
                custOctree[i * octree_size + j].max = oct.Maximum(oct.center, custBB[i].size / 4);
            }
        }
    }

    private void AddTriangles(int i)
    {
        var posTri = deformableCPUMSM[i].GetPosTriangles();
        for(int j=0; j< triCount; j++)
        {
            posTriangle.Add(new Tris
            {
                vertex0 = posTri[j].vertex0,
                vertex1 = posTri[j].vertex1,
                vertex2 = posTri[j].vertex2
            });
        }
    }

    private int calcIndexObjectLevel0(int object_index)
    {
        return (int)Mathf.Floor(object_index / octree_size) * octree_size;
    }
    private void CheckTriIntersection() 
    {
        collidableTriIndex.Clear();
        collidablePairIndexL0.Clear();
        collidablePairIndexL1.Clear();
        for(int i =0; i< number_object * triCount;i++)
        {
            for(int j =0; j< number_object * triCount;j++)
            {
                int objIndex1 = (int)Mathf.Floor(i/triCount);
                int objIndex2 = (int)Mathf.Floor(j/triCount);

                if(objIndex1 != objIndex2 && objIndex1 >= objIndex2)
                {
                    for (int p = 0; p < pairIndexL1.Count; p++)
                    {
                        int l0_index_obj1 = calcIndexObjectLevel0(pairIndexL1[p].i1);
                        int l0_index_obj2 = calcIndexObjectLevel0(pairIndexL1[p].i2);

                        // check collision box of object, before check tri-intersection
                        if (Intersection.AABB(custOctree[l0_index_obj1].min, custOctree[l0_index_obj1].max,
                        custOctree[l0_index_obj2].min, custOctree[l0_index_obj2].max))
                        {                               
                            if (debugModeLv0)
                            {
                                if (!collidablePairIndexL0.Contains(l0_index_obj1))
                                    collidablePairIndexL0.Add(l0_index_obj1);

                                if (!collidablePairIndexL0.Contains(l0_index_obj2))
                                    collidablePairIndexL0.Add(l0_index_obj2);
                            }

                            if (Intersection.AABB(custOctree[pairIndexL1[p].i1].min, custOctree[pairIndexL1[p].i1].max,
                            custOctree[pairIndexL1[p].i2].min, custOctree[pairIndexL1[p].i2].max))
                            {
                                if (debugModeLv1)
                                {
                                    if (!collidablePairIndexL1.Contains(pairIndexL1[p].i1))
                                        collidablePairIndexL1.Add(pairIndexL1[p].i1);

                                    if (!collidablePairIndexL1.Contains(pairIndexL1[p].i2))
                                        collidablePairIndexL1.Add(pairIndexL1[p].i2);
                                }

                                var t1 = posTriangle[i];
                                var t2 = posTriangle[j];

                                if (Detection(t1, t2))
                                {                   
                                    if(debugModeTriangle)
                                    {
                                        Vector2Int pair = new Vector2Int(i, j);
                                        if(!collidableTriIndex.Contains(pair)){
                                            collidableTriIndex.Add(pair); 
                                        } 
                                    }                                  
                                }
                            }
                        
                        }
                    }                        
                    
                }
            }
        }  
    }

    bool Detection(Tris t1, Tris t2)
    {
        var c1 = CheckEdgeCollision(t1.vertex0, t1.vertex1, t2) || 
        CheckEdgeCollision(t1.vertex0, t1.vertex2, t2) || 
        CheckEdgeCollision(t1.vertex1, t1.vertex2, t2);

        var c2 = CheckEdgeCollision(t2.vertex0, t2.vertex1, t1) || 
        CheckEdgeCollision(t2.vertex0, t2.vertex2, t1) || 
        CheckEdgeCollision(t2.vertex1, t2.vertex2, t1);

        return c1 && c2;
    }


    Vector3 ProjectPointOnPlane(Vector3 point, Vector3 planeNormal, Vector3 planePoint)
    {
        float d = Vector3.Dot(planeNormal, (point - planePoint)) / planeNormal.magnitude;
        return point - d * planeNormal;
    }


    bool IsPointInsideTriangle(Vector3 point, Tris triangle)
    {
         Vector3 normal = Vector3.Cross(triangle.vertex1 - triangle.vertex0, triangle.vertex2 - triangle.vertex0).normalized;

        // ���� �ﰢ�� ��鿡 ����
        Vector3 projectedPoint = ProjectPointOnPlane(point, normal, triangle.vertex0);

        if (Vector3.Distance(projectedPoint, point) > 0.1) return false;

        //Debug.Log(Vector3.Distance(projectedPoint, point));

        // ������ ���� ���� ���� �Ǵ� ����
        Vector3 edge1 = triangle.vertex1 - triangle.vertex0;
        Vector3 vp1 = projectedPoint - triangle.vertex0;
        if (Vector3.Dot(Vector3.Cross(edge1, vp1), normal) < 0) return false;

        Vector3 edge2 = triangle.vertex2 - triangle.vertex1;
        Vector3 vp2 = projectedPoint - triangle.vertex1;
        if (Vector3.Dot(Vector3.Cross(edge2, vp2), normal) < 0) return false;

        Vector3 edge3 = triangle.vertex0 - triangle.vertex2;
        Vector3 vp3 = projectedPoint - triangle.vertex2;
        if (Vector3.Dot(Vector3.Cross(edge3, vp3), normal) < 0) return false;

        return true; // ��� �˻縦 ����ߴٸ�, ������ ���� �ﰢ�� ���ο� �ֽ��ϴ�.
    }

    Vector3 FindClosestVertex(Tris triangle, Vector3 point)
    {
        float minDistance = Mathf.Infinity;
        Vector3 closestVertex = Vector3.zero;

        float distance0 = Vector3.Distance(triangle.vertex0, point);
        float distance1 = Vector3.Distance(triangle.vertex1, point);
        float distance2 = Vector3.Distance(triangle.vertex2, point);

        if (distance0 < minDistance)
        {
            minDistance = distance0;
            closestVertex = triangle.vertex0;
        }
        if (distance1 < minDistance)
        {
            minDistance = distance1;
            closestVertex = triangle.vertex1;
        }
        if (distance2 < minDistance)
        {
            minDistance = distance2;
            closestVertex = triangle.vertex2;
        }

        return closestVertex;
    }

    bool checkPointCollision(Vector3 p, Tris triangle)
    {
        return IsPointInsideTriangle(p, triangle);
    }

    bool CheckEdgeCollision(Vector3 vertex1, Vector3 vertex2, Tris t)
    {
        var edge = new Line();

        edge.p0 = vertex1;
        edge.p1 = vertex2;

        return Intersect(t, edge, ref hitPoint);
    }

    public bool Intersect(Tris triangle, Line ray, ref Vector3 hit)
    {
        // Vectors from p1 to p2/p3 (edges)
        //Find vectors for edges sharing vertex/point p1
        Vector3 e1 = triangle.vertex1 - triangle.vertex0;
        Vector3 e2 = triangle.vertex2 - triangle.vertex0;

        // Calculate determinant
        Vector3 p = Vector3.Cross(ray.direction, e2);

        //Calculate determinat
        float det = Vector3.Dot(e1, p);

        //if determinant is near zero, ray lies in plane of triangle otherwise not
        if (det > -Mathf.Epsilon && det < Mathf.Epsilon)
        {
            var coplanar = IsPointInsideTriangle(ray.p0, triangle);
            var coplanar2 = IsPointInsideTriangle(ray.p1, triangle);

            if (coplanar) hit = ray.p0;
            if (coplanar2) hit = ray.p1;

            return coplanar || coplanar2;
        }
        float invDet = 1.0f / det;

        //calculate distance from p1 to ray origin
        Vector3 t = ray.origin - triangle.vertex0;

        //Calculate u parameter
        float u = Vector3.Dot(t, p) * invDet;

        //Check for ray hit
        if (u < 0 || u > 1) { return false; }

        //Prepare to test v parameter
        Vector3 q = Vector3.Cross(t, e1);

        //Calculate v parameter
        float v = Vector3.Dot(ray.direction, q) * invDet;

        //Check for ray hit
        if (v < 0 || u + v > 1) { return false; }

        // intersection point
        hit = triangle.vertex0 + u * e1 + v * e2;

        if ((Vector3.Dot(e2, q) * invDet) > Mathf.Epsilon)
        {
            //ray does intersect            
            return true;
        }

        // No hit at all
        return false;
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
        if (custOctree != null )
        {
            for (int i = 0; i < custOctree.Length; i++)
            {
                if (debugModeLv0)
                {
                    Gizmos.color = Color.red;
                    for (int j = 0; j < collidablePairIndexL0.Count; j++)
                    {
                        if (i == collidablePairIndexL0[j])
                        {
                            Gizmos.DrawWireCube(custOctree[collidablePairIndexL0[j]].center, custOctree[collidablePairIndexL0[j]].size);
                        }
                    }
                }
               
                if (debugModeLv1)
                {
                    Gizmos.color = Color.green;
                    for (int j = 0; j < collidablePairIndexL1.Count; j++)
                    {
                        if (i == collidablePairIndexL1[j])
                        {
                            Vector3 size = custOctree[collidablePairIndexL1[j]].max - custOctree[collidablePairIndexL1[j]].min;
                            Gizmos.DrawWireCube(custOctree[collidablePairIndexL1[j]].center, size);
                        }
                    }
                }
            }
        }

        if (collidableTriIndex != null && debugModeTriangle)
        {
            List<int> newList = new List<int>();
            for(int i =0; i< collidableTriIndex.Count; i++){
                if(!newList.Contains(collidableTriIndex[i].x)) newList.Add(collidableTriIndex[i].x);
                if(!newList.Contains(collidableTriIndex[i].y)) newList.Add(collidableTriIndex[i].y);
            }

            for(int i =0; i< newList.Count; i++)
            {
                DrawTriangle(posTriangle[newList[i]], Color.green);
            }
        }
    }

     private void DrawTriangle(Tris triangle, Color color)
    {
        Gizmos.color = color;
        Gizmos.DrawLine(triangle.vertex0, triangle.vertex1);
        Gizmos.DrawLine(triangle.vertex1, triangle.vertex2);
        Gizmos.DrawLine(triangle.vertex2, triangle.vertex0);
    }
}
