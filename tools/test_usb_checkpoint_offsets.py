"""Focused checks of offset geometry and the bounded continuous fit."""
import unittest
import numpy as np
from fit_usb_checkpoint_offsets import replay_basis, bounded_fit


class GeometryTests(unittest.TestCase):
    def test_rotation_about_fixed_point(self):
        h=np.linspace(0,1.9,143)**1.1
        basis=replay_basis(-20*h,-70*h,h)
        xy=basis[:,:2]+20*basis[:,2:4]-70*basis[:,4:6]
        np.testing.assert_allclose(xy,0,atol=1e-11)

    def test_full_turn_offset_cancellation_even_without_reversing(self):
        h=np.linspace(0,2*np.pi,601)
        # Constant positive forward speed and clockwise turning: a closed circle.
        basis=replay_basis(100*h-20*h,-70*h,h)
        for f,l in ((20,-70),(300,-200),(-500,500)):
            np.testing.assert_allclose(basis[-1,:2]+f*basis[-1,2:4]+l*basis[-1,4:6],0,atol=1e-10)

    def test_fit_recovers_offsets_with_rotated_references(self):
        A=np.array([[1,-1],[1,1],[2,0],[0,2]],dtype=float)
        truth=np.array([20,-70.])
        raw,bounded,rank,_=bounded_fit(A,A@truth,-500,500)
        self.assertEqual(rank,2)
        np.testing.assert_allclose(raw,truth,atol=1e-10)
        np.testing.assert_allclose(bounded,truth,atol=1e-10)

    def test_bounded_fit_and_unobservable_design(self):
        A=np.eye(2)
        _,bounded,_,_=bounded_fit(A,np.array([900.,-600]),-500,500)
        np.testing.assert_allclose(bounded,[500,-500])
        _,_,rank,singular=bounded_fit(np.zeros((4,2)),np.array([30,10,20,5.]),-500,500)
        self.assertEqual(rank,0)
        np.testing.assert_array_equal(singular,0)


if __name__=="__main__":unittest.main()
