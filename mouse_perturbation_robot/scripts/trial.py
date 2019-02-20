
import numpy as np
from sklearn.linear_model import LogisticRegression
import matplotlib.pyplot as plt
from iteration_learning import IterationLearning
from numpy.polynomial.polynomial import polyval
from sklearn.datasets import make_classification
import time


# X, y = make_classification(200, 2, 2, 0, weights=[.5, .5], random_state=15)

clf = LogisticRegression().fit([[0,0],[10,0]], [0,1])
print(clf.coef_)

if 0:
    X = np.array([[0, 0.9],
                  [8, 1.6]])
    y = np.array([0, 1])
    clf = LogisticRegression().fit(X, y)

    xx, yy = np.mgrid[0:8:.01, 0.9:1.6:.01]
    grid = np.c_[xx.ravel(), yy.ravel()]
    probs = clf.predict_proba(grid)[:, 1].reshape(xx.shape)

    f, ax = plt.subplots(figsize=(8, 6))
    contour = ax.contourf(xx, yy, probs, 25, cmap="RdBu",
                          vmin=0, vmax=1)
    # contour = ax.contourf(xx, yy, probs, levels=[.5], cmap="Greys",
    #                      vmin=0, vmax=0.6)
    ax_c = f.colorbar(contour)
    ax_c.set_label("$P(y = 1)$")
    ax_c.set_ticks([0, .25, .5, .75, 1])

    ax.scatter(X[:, 0], X[:, 1], c=y, s=50,
               cmap="RdBu", vmin=-.2, vmax=1.2,
               edgecolor="white", linewidth=1)

    ax.set(aspect="auto",
           xlim=(0, 8), ylim=(0.9, 1.6),
           xlabel="$X_1$", ylabel="$X_2$")

    plt.show()
else:

    # define a ground truth
    def ground_truth(x):
        para = [1.6, -1/6]
        if x[1] > polyval(x[0], para):
            return 1
        else:
            return 0

    # start the iteration...
    il = IterationLearning([0, 8], [0.9, 1.6])
    X = np.array([[0, 0.9],
                  [8, 1.6]])
    y = np.array([0, 1])

    il.two_init_point(X, y)
    i = 0

    trail = il.update_data_x()
    while i < 100:

        # print(trail)

        # print(ground_truth(trail))

        trail = il.run(ground_truth(trail))
        i += 1
        time.sleep(0.1)
