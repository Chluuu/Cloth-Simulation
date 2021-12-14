using UnityEngine;
using System.Collections;

public class PBD_model: MonoBehaviour {

	float 		t= 0.0333f;
	float		damping= 0.99f;
	int[] 		E;
	float[] 	L;
	Vector3[] 	V;
	int			n = 21;

	ComputeBuffer vertPos;
	ComputeBuffer vertVel;
	ComputeBuffer vertMove;
	ComputeBuffer moveCount;

	[SerializeField] ComputeShader _compute;

	static class Kernels{
		public const int SetUp=0;
		public const int Blending=1;
		public const int Collision=2;
	}

	int BlockCount { get { return (n + 7) / 8; } }
	int BlockCountEdge;

	void OnDestroy(){
		vertPos.Release();
		vertVel.Release();
		vertMove.Release();
		moveCount.Release();
	}

	// Use this for initialization
	void Start () 
	{
		Mesh mesh = GetComponent<MeshFilter> ().mesh;

		//Resize the mesh.
		Vector3[] X  	= new Vector3[n*n];
		Vector2[] UV 	= new Vector2[n*n];
		int[] T	= new int[(n-1)*(n-1)*6];

		vertPos=new ComputeBuffer(n*n, sizeof(float)*3);
		vertVel=new ComputeBuffer(n*n, sizeof(float)*3);
		vertMove=new ComputeBuffer(n*n, sizeof(float)*3);
		moveCount=new ComputeBuffer(n*n, sizeof(float));

		for(int j=0; j<n; j++)
		for(int i=0; i<n; i++)
		{
			X[j*n+i] =new Vector3(5-10.0f*i/(n-1), 0, 5-10.0f*j/(n-1));
			UV[j*n+i]=new Vector3(i/(n-1.0f), j/(n-1.0f));
		}
		int t=0;
		for(int j=0; j<n-1; j++)
		for(int i=0; i<n-1; i++)	
		{
			T[t*6+0]=j*n+i;
			T[t*6+1]=j*n+i+1;
			T[t*6+2]=(j+1)*n+i+1;
			T[t*6+3]=j*n+i;
			T[t*6+4]=(j+1)*n+i+1;
			T[t*6+5]=(j+1)*n+i;
			t++;
		}
		mesh.vertices	= X;
		mesh.triangles	= T;
		mesh.uv 		= UV;
		mesh.RecalculateNormals ();

		//Construct the original edge list
		int[] _E = new int[T.Length*2];
		for (int i=0; i<T.Length; i+=3) 
		{
			_E[i*2+0]=T[i+0];
			_E[i*2+1]=T[i+1];
			_E[i*2+2]=T[i+1];
			_E[i*2+3]=T[i+2];
			_E[i*2+4]=T[i+2];
			_E[i*2+5]=T[i+0];
		}
		//Reorder the original edge list
		for (int i=0; i<_E.Length; i+=2)
			if(_E[i] > _E[i + 1]) 
				Swap(ref _E[i], ref _E[i+1]);
		//Sort the original edge list using quicksort
		Quick_Sort (ref _E, 0, _E.Length/2-1);

		int e_number = 0;
		for (int i=0; i<_E.Length; i+=2)
			if (i == 0 || _E [i + 0] != _E [i - 2] || _E [i + 1] != _E [i - 1]) 
				e_number++;

		E = new int[e_number * 2];
		for (int i=0, e=0; i<_E.Length; i+=2)
			if (i == 0 || _E [i + 0] != _E [i - 2] || _E [i + 1] != _E [i - 1]) 
			{
				E[e*2+0]=_E [i + 0];
				E[e*2+1]=_E [i + 1];
				e++;
			}

		L = new float[E.Length/2];
		for (int e=0; e<E.Length/2; e++) 
		{
			int i = E[e*2+0];
			int j = E[e*2+1];
			L[e]=(X[i]-X[j]).magnitude;
		}

		V = new Vector3[X.Length];
		for (int i=0; i<X.Length; i++)
			V[i] = new Vector3 (0, 0, 0);

		vertPos.SetData(X);
		vertVel.SetData(V);
	}

	void Quick_Sort(ref int[] a, int l, int r)
	{
		int j;
		if(l<r)
		{
			j=Quick_Sort_Partition(ref a, l, r);
			Quick_Sort (ref a, l, j-1);
			Quick_Sort (ref a, j+1, r);
		}
	}

	int  Quick_Sort_Partition(ref int[] a, int l, int r)
	{
		int pivot_0, pivot_1, i, j;
		pivot_0 = a [l * 2 + 0];
		pivot_1 = a [l * 2 + 1];
		i = l;
		j = r + 1;
		while (true) 
		{
			do ++i; while( i<=r && (a[i*2]<pivot_0 || a[i*2]==pivot_0 && a[i*2+1]<=pivot_1));
			do --j; while(  a[j*2]>pivot_0 || a[j*2]==pivot_0 && a[j*2+1]> pivot_1);
			if(i>=j)	break;
			Swap(ref a[i*2], ref a[j*2]);
			Swap(ref a[i*2+1], ref a[j*2+1]);
		}
		Swap (ref a [l * 2 + 0], ref a [j * 2 + 0]);
		Swap (ref a [l * 2 + 1], ref a [j * 2 + 1]);
		return j;
	}

	void Swap(ref int a, ref int b)
	{
		int temp = a;
		a = b;
		b = temp;
	}

	void Strain_Limiting()
	{
		Mesh mesh = GetComponent<MeshFilter> ().mesh;
		Vector3[] vertices = mesh.vertices;

		Vector3[] moves=new Vector3[n*n];
		float[] moveN=new float[n*n];
		//Apply PBD here.
		for(int e=0;e<E.Length/2;e++)
		{
			int i = E[e*2+0];
			int j = E[e*2+1];
			float distance=(vertices[i]-vertices[j]).magnitude;
			moves[i]+=0.5f*(vertices[i]+vertices[j]+L[e]*(vertices[i]-vertices[j])/distance);
			moveN[i]++;
			moves[j]+=0.5f*(vertices[i]+vertices[j]-L[e]*(vertices[i]-vertices[j])/distance);
			moveN[j]++;
		}

		vertPos.SetData(vertices);
		vertMove.SetData(moves);
		moveCount.SetData(moveN);

		_compute.SetBuffer(Kernels.Blending, "_Moves", vertMove);
		_compute.SetBuffer(Kernels.Blending, "_MoveCount", moveCount);
		_compute.SetBuffer(Kernels.Blending, "_Pos", vertPos);
		_compute.SetBuffer(Kernels.Blending, "_Vel", vertVel);
		_compute.Dispatch(Kernels.Blending, BlockCount, BlockCount, 1);

		vertPos.GetData(vertices);
		vertVel.GetData(V);

		mesh.vertices = vertices;
	}

	void Collision_Handling()
	{
		Mesh mesh = GetComponent<MeshFilter> ().mesh;
		Vector3[] X = mesh.vertices;
		
		//Handle colllision.
		GameObject sphere=GameObject.Find("Sphere");
		Vector3 center=sphere.transform.position;

		_compute.SetVector("_SphereCenter", center);
		_compute.SetBuffer(Kernels.Collision, "_Pos", vertPos);
		_compute.SetBuffer(Kernels.Collision, "_Vel", vertVel);
		_compute.Dispatch(Kernels.Collision, BlockCount, BlockCount, 1);
		vertPos.GetData(X);
		vertVel.GetData(V);

		mesh.vertices = X;
	}

	// Update is called once per frame
	void Update () 
	{
		Mesh mesh = GetComponent<MeshFilter> ().mesh;
		Vector3[] X = mesh.vertices;

		//Initial Setup.
		vertPos.SetData(X);
		_compute.SetVector("_Gravity",new Vector3(0,-9.98f,0));
		_compute.SetFloat("_DeltaTime", t);
		_compute.SetFloat("_Size", n);
		_compute.SetFloat("_Damping", damping);
		_compute.SetBuffer(Kernels.SetUp, "_Pos", vertPos);
		_compute.SetBuffer(Kernels.SetUp, "_Vel", vertVel);
		_compute.Dispatch(Kernels.SetUp, BlockCount, BlockCount, 1);
		vertPos.GetData(X);

		mesh.vertices = X;

		for(int l=0; l<32; l++)
			Strain_Limiting ();

		Collision_Handling ();

		mesh.RecalculateNormals ();

	}


}

