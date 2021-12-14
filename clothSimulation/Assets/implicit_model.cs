using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class implicit_model : MonoBehaviour
{
	float 		t 		= 0.0333f;
	float 		mass	= 1;
	float		damping	= 0.99f;
	float 		rho		= 0.995f;
	float 		spring_k = 8000;
	int[] 		E;
	float[] 	L;
	Vector3[] 	V;
	int			n = 21;

	ComputeBuffer vertPos;
	ComputeBuffer vertPosTilde;
	ComputeBuffer vertVel;
	ComputeBuffer vertGra;
	ComputeBuffer vertPosLast;

	[SerializeField] ComputeShader _compute;

	static class Kernels{
		public const int SetUp=0;
		public const int CalGradientElement=1;
		public const int UpdateX=2;
		public const int UpdateV=3;
		public const int Collision=4;
		public const int Test=5;
	}

	int BlockCount { get { return (n + 7) / 8; } }

	void OnDestroy(){
		vertPos.Release();
		vertVel.Release();
		vertPosTilde.Release();
		vertGra.Release();
		vertPosLast.Release();
	}

    // Start is called before the first frame update
    void Start()
    {
		Mesh mesh = GetComponent<MeshFilter> ().mesh;

		//Resize the mesh.		
		Vector3[] X  	= new Vector3[n*n];
		Vector2[] UV 	= new Vector2[n*n];
		int[] triangles	= new int[(n-1)*(n-1)*6];

		vertPos=new ComputeBuffer(n*n, sizeof(float)*3);
		vertVel=new ComputeBuffer(n*n, sizeof(float)*3);
		vertPosTilde=new ComputeBuffer(n*n, sizeof(float)*3);
		vertGra=new ComputeBuffer(n*n, sizeof(float)*3);
		vertPosLast=new ComputeBuffer(n*n, sizeof(float)*3);

		Vector3 test=new Vector3(1,2,3.3f);
		
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
			triangles[t*6+0]=j*n+i;
			triangles[t*6+1]=j*n+i+1;
			triangles[t*6+2]=(j+1)*n+i+1;
			triangles[t*6+3]=j*n+i;
			triangles[t*6+4]=(j+1)*n+i+1;
			triangles[t*6+5]=(j+1)*n+i;
			t++;
		}
		mesh.vertices=X;
		mesh.triangles=triangles;
		mesh.uv = UV;
		mesh.RecalculateNormals ();

		//Construct the original E
		int[] _E = new int[triangles.Length*2];
		for (int i=0; i<triangles.Length; i+=3) 
		{
			_E[i*2+0]=triangles[i+0];
			_E[i*2+1]=triangles[i+1];
			_E[i*2+2]=triangles[i+1];
			_E[i*2+3]=triangles[i+2];
			_E[i*2+4]=triangles[i+2];
			_E[i*2+5]=triangles[i+0];
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
			int v0 = E[e*2+0];
			int v1 = E[e*2+1];
			L[e]=(X[v0]-X[v1]).magnitude;
		}

		V = new Vector3[X.Length];
		for (int i=0; i<V.Length; i++)
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

	void Get_Gradient(Vector3[] X, Vector3[] X_hat, float t, Vector3[] G)
	{
		
		//Momentum and Gravity.
		_compute.SetBuffer(Kernels.CalGradientElement, "_Pos", vertPos);
		_compute.SetBuffer(Kernels.CalGradientElement, "_PosTilde", vertPosTilde);
		_compute.SetBuffer(Kernels.CalGradientElement, "_Gra", vertGra);
		_compute.Dispatch(Kernels.CalGradientElement, BlockCount, BlockCount, 1);

		vertGra.GetData(G);
		
		//Spring Force.
		
		for(int e=0;e<E.Length/2;e++)
		{
			int v0 = E[e*2+0];
			int v1 = E[e*2+1];

			float newLength=(X[v0]-X[v1]).magnitude;
			Vector3 update=spring_k*(1.0f-L[e]/newLength)*(X[v0]-X[v1]);
			G[v0]=G[v0]+update;
			G[v1]=G[v1]-update;
		}
		vertGra.SetData(G);
		
	}

    // Update is called once per frame
	void Update () 
	{
		Mesh mesh = GetComponent<MeshFilter> ().mesh;
		Vector3[] X 		= mesh.vertices;
		Vector3[] last_X 	= new Vector3[X.Length];
		Vector3[] X_hat 	= new Vector3[X.Length];
		Vector3[] G 		= new Vector3[X.Length];
		
		//Initial Setup.
		vertPos.SetData(X);
		_compute.SetVector("_Gravity",new Vector3(0,-9.98f,0));
		_compute.SetFloat("_DeltaTime", t);
		_compute.SetFloat("_Size", n);
		_compute.SetFloat("_Damping", damping);
		_compute.SetBuffer(Kernels.SetUp, "_Pos", vertPos);
		_compute.SetBuffer(Kernels.SetUp, "_PosTilde", vertPosTilde);
		_compute.SetBuffer(Kernels.SetUp, "_Vel", vertVel);
		_compute.Dispatch(Kernels.SetUp, BlockCount, BlockCount, 1);
		vertPosTilde.GetData(X_hat);
		vertPos.GetData(X);
		
		float w=1;
		for(int k=0; k<32; k++)
		{
			Get_Gradient(X, X_hat, t, G);

			//Update X by gradient.
        	if(k==1)w=2.0f/(2.0f-rho*rho);
        	if(k>1)w=4.0f/(4.0f-rho*rho*w);
			_compute.SetFloat("_W",w);
			_compute.SetFloat("_K",spring_k);
			_compute.SetBuffer(Kernels.UpdateX, "_PosLast", vertPosLast);
			_compute.SetBuffer(Kernels.UpdateX, "_Pos", vertPos);
			_compute.SetBuffer(Kernels.UpdateX, "_Gra", vertGra);			
			_compute.Dispatch(Kernels.UpdateX, BlockCount, BlockCount, 1);
			vertPos.GetData(X);
		}
		
		//Finishing.
		_compute.SetBuffer(Kernels.UpdateV, "_Pos", vertPos);
		_compute.SetBuffer(Kernels.UpdateV, "_PosTilde", vertPosTilde);			
		_compute.SetBuffer(Kernels.UpdateV, "_Vel", vertVel);
		_compute.Dispatch(Kernels.UpdateV, BlockCount, BlockCount, 1);
		
		vertPos.GetData(X);
		vertVel.GetData(V);

		mesh.vertices = X;

		Collision_Handling ();
		mesh.RecalculateNormals ();
	}
}


