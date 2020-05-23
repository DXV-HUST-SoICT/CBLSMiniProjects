import com.google.ortools.linearsolver.MPConstraint;
import com.google.ortools.linearsolver.MPObjective;
import com.google.ortools.linearsolver.MPSolver;
import com.google.ortools.linearsolver.MPVariable;
import com.google.ortools.linearsolver.MPSolver.ResultStatus;

public class SportScheduling {
	static {
		System.loadLibrary("jniortools");
	}
	int T = 4;
	int[][] d = {{ 0, 3, 2, 4 },
			{ 9, 0, 2, 3 },
			{ 1, 2, 0, 7 },
			{ 1, 1, 4, 0 }};
	int W = 2 * T - 2;

	MPVariable[][][] X;
	MPVariable[][][][] F;
	MPSolver solver;

	public void buildModel() {
		solver = new MPSolver("SportScheduling",
				MPSolver.OptimizationProblemType.CBC_MIXED_INTEGER_PROGRAMMING);
		X = new MPVariable[T][T][W + 2];
		F = new MPVariable[T][T][T][W + 1];

		for (int i = 0; i < T; i++) {
			for (int j = 0; j < T; j++) {
				for (int t = 0; t <= W + 1; t++) {
					X[i][j][t] = solver.makeIntVar(0, 1, "X_" + i + "_" + j + "_" + t);
					if (t == 0 || t == W + 1) {
						if (i == j) {
							MPConstraint c = solver.makeConstraint(1, 1);
							c.setCoefficient(X[i][j][t], 1);
						} else {
							MPConstraint c = solver.makeConstraint(0, 0);
							c.setCoefficient(X[i][j][t], 1);
						}
					}
				}
			}
			for (int u = 0; u < T; u++) {
				for (int v = 0; v < T; v++) {
					for (int t = 0; t <= W; t++) {
						F[i][u][v][t] = solver.makeIntVar(0, 1, "F_" + i + "_" + u + "_" + v + "_" + t);
					}
				}
			}
		}

		for (int i = 0; i < T; i++) {
			for (int t = 1; t <= W; t++) {

				MPConstraint c = solver.makeConstraint(1, 1);
				for (int j = 0; j < T; j++) {
					if (i != j) {
						c.setCoefficient(X[i][j][t], 1);
						c.setCoefficient(X[j][i][t], 1);
					}
				}

				MPConstraint c1 = solver.makeConstraint(0, 0);
				c1.setCoefficient(X[i][i][t], 1);
				for (int j = 0; j < T; j++) {
					if (j != i) {
						c1.setCoefficient(X[i][j][t], -1);
					}
				}
			}
		}

		for (int i = 0; i < T; i++) {
			for (int j = 0; j < T; j++) {
				if (i != j) {
					MPConstraint c = solver.makeConstraint(1, 1);
					for (int t = 1; t <= W; t++) {
						c.setCoefficient(X[i][j][t], 1);
					}
				}
			}
		}

		for (int i = 0; i < T; i++) {
			for (int u = 0; u < T; u++) {
				for (int v = 0; v < T; v++) {
					for (int t = 0; t < W + 1; t++) {
						MPConstraint c = solver.makeConstraint(-1, solver.infinity());
						c.setCoefficient(F[i][u][v][t], 1);
						c.setCoefficient(X[u][i][t], -1);
						c.setCoefficient(X[v][i][t+1], -1);
					}
				}
			}
		}

		MPObjective o = solver.objective();
		for (int i = 0; i < T; i++) {
			for (int u = 0; u < T; u++) {
				for (int v = 0; v < T; v++) {
					for (int t = 0; t <= W; t++) {
						o.setCoefficient(F[i][u][v][t], d[u][v]);
					}
				}
			}
		}
		o.setMinimization();

		ResultStatus rs = solver.solve();
		if (rs != ResultStatus.OPTIMAL) {
			System.out.println("cannot find optimal solution");
		} else {
			System.out.println("obj= " + o.value());
			printSol();
		}
	}

	private void printSol() {
//		System.out.println("-------------");
//		System.out.println("#" + 0);
//		for (int i = 0; i < T; i++) {
//			for (int u = 0; u < T; u++) {
//				for (int v = 0; v < T; v++) {
//					if (F[i][u][v][0].solutionValue() == 1) {
//						System.out.println(i + ": " + u + " -> " + v);
//					}
//				}
//			}
//		}

		for (int t = 1; t <= W; t++) {
			System.out.println("#" + t);
			for (int i = 0; i < T; i++) {
				for (int j = 0; j < T; j++)
					if (i != j) {
						if (X[i][j][t].solutionValue() > 0)
							System.out.println("X(" + i + "," + j + "," + t
									+ ") -> " + i + " vs " + j + " san " + i);
					}
			}
			System.out.println("-------------");

//			for (int i = 0; i < T; i++) {
//				for (int u = 0; u < T; u++) {
//					for (int v = 0; v < T; v++) {
//						if (F[i][u][v][t].solutionValue() == 1) {
//							System.out.println(i + ": " + u + " -> " + v);
//						}
//					}
//				}
//			}
		}

		for (int j = 0; j < T; j++) {
			System.out.print(j + ": " + j);
			for (int t = 1; t <= W; t++) {
				for (int i = 0; i < T; i++) {
					if (X[i][j][t].solutionValue() == 1) {
						System.out.print(" -> " + i);
						break;
					}
				}
			}
			System.out.println(" -> " + j);
		}

//		int obj = 0;
//
//		for (int i = 0; i < T; i++) {
//			for (int u = 0; u < T; u++) {
//				for (int v = 0; v < T; v++) {
//					for (int t = 0; t <= W; t++) {
//						if (F[i][u][v][t].solutionValue() == 1) {
//							obj += d[u][v];
//						}
//					}
//				}
//			}
//		}
//
//		System.out.println("Validation objective: " + obj);
	}

	public static void main(String[] args) {
		// TODO Auto-generated method stub
		SportScheduling app = new SportScheduling();
		app.buildModel();
	}

}
