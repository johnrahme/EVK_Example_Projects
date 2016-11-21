import com.lemmingapex.trilateration.TrilaterationTest;

public class Test {

	public static void main(String[] args) {
		System.out.println("helloAgain");
		TrilaterationTest test = new TrilaterationTest();
		try {
			double[][] positions = new double[][] { { 1.0, 1.0 }, { 3.0, 1.0 }, { 2.0, 2.0 } };
			double[] distances = new double[] { 0.9, 1.0, 1.0 };
			test.trilateration2DInexact1(positions, distances);
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}


	}

}
