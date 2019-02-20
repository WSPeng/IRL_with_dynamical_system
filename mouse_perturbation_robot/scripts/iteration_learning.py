#!/usr/bin/env python

import numpy as np
import random
from numpy.polynomial.polynomial import polyval
from numpy.polynomial.polynomial import polyroots
import matplotlib.pyplot as plt
import scipy
from sklearn import svm
from sklearn.linear_model import LogisticRegression


class IterationLearning:

    def __init__(self, x_range, y_range):
        """
        x_rang : [low_limit, high_limit] -- [list]
        """
        self.x_range = x_range
        self.y_range = y_range

        # init the data X and Y
        self.X = np.array([])  # N x 2
        self.Y = np.array([])  # N x 1

        # initialize the fitting, which is empty
        self.para = np.array([])

        # init plot
        self.figure, self.ax = plt.subplots(figsize=(8, 6))
        plt.ion()

        # other
        self.x_sample = 0.0
        self.y_sample = 0.0
        self.deviation = 0.0

        # TODO: how to init the clf
        # self.clf = LogisticRegression()

    def _check_within_boundary(self, x, y):
        """ check if the point (x,y) falls inside the boundary. """
        if all([self.x_range[0] < x < self.x_range[1], self.y_range[0] < y < self.y_range[1]]):
            return True
        else:
            return False

    def _uniform_sample_on_curve(self):
        """ start with 1st order polynomial
            return one sample
        """
        # calculate the intersection boudnary
        # only valid for striaght line, since only one intersection.
        b_0 = self.para.copy()
        b_1 = self.para.copy()
        b_0[0] = b_0[0] - self.y_range[0]
        b_1[0] = b_1[0] - self.y_range[1]

        root_0 = polyroots(b_0)[0]
        root_1 = polyroots(b_1)[0]

        # calculate the intersection of two range
        roots = [root_0, root_1]
        roots.sort()
        if roots[0] > self.x_range[1] or roots[1] < self.x_range[0] or (roots[0] < self.x_range[0] and roots[1] > self.x_range[1]):
            limit_0 = self.x_range[0]
            limit_1 = self.x_range[1]
        elif roots[0] < self.x_range[0] and roots[1] < self.x_range[1]:
            limit_0 = self.x_range[0]
            limit_1 = roots[1]
        elif roots[0] > self.x_range[0] and roots[1] > self.x_range[1]:
            limit_0 = roots[0]
            limit_1 = self.x_range[1]
        elif roots[0] > self.x_range[0] and roots[1] < self.x_range[1]:
            limit_0 = roots[0]
            limit_1 = roots[1]

        # uniform sample on x
        self.x_sample = random.uniform(limit_0, limit_1)
        self.y_sample = polyval(self.x_sample, self.para)
        # the boundary check will be performed after gaussian distance

    def _gaussian_deviation(self):
        """ """
        # gaussian random generate the bias distance
        # self.deviation = random.gauss(0, 0.5)
        self.deviation = 0.0
        # print(self.deviation)

    def two_init_point(self, x, y):
        """ """
        # coef_ only return the weight of x and y, need to translate to y = c_0 + c_1*x format
        self.clf = LogisticRegression().fit(x, y)
        w = self.clf.coef_[0]
        c_1 = -w[0] / w[1]
        intercept = - self.clf.intercept_[0] / w[1]
        self.para = np.array([intercept, c_1])
        self.X = x
        self.Y = y

    def update_data_x(self):
        """ """
        # 0 on denominator
        # slope is the perpendicular line slope
        if self.para[1] != 0:
            slope = -1/self.para[1]
        else:
            slope = 1  # TODO 1?

        self._uniform_sample_on_curve()
        self._gaussian_deviation()
        x = self.x_sample + self.deviation * 1 / np.sqrt(1+slope**2)
        y = self.y_sample + self.deviation * slope / np.sqrt(1+slope**2)

        while not self._check_within_boundary(x, y):
            # generate new random parameters
            self._uniform_sample_on_curve()
            self._gaussian_deviation()
            # update new point base the new random parameters
            x = self.x_sample + self.deviation * 1 / np.sqrt(1 + slope**2)
            y = self.y_sample + self.deviation * slope / np.sqrt(1 + slope**2)

        self.X = np.append(self.X, np.array([[x, y]]), axis=0)

        return np.array([x, y])

    def update_guess(self, result):
        """ """
        # update Y
        self.Y = np.append(self.Y, result)

        # update the new guess decision boundary
        self.clf = LogisticRegression().fit(self.X, self.Y)

        w = self.clf.coef_[0]
        c_1 = -w[0] / w[1]
        intercept = - self.clf.intercept_[0] / w[1]
        self.para = np.array([intercept, c_1])

    def _figure_update(self):
        """ """
        xx, yy = np.mgrid[self.x_range[0]:self.x_range[1]:.01, self.y_range[0]:self.y_range[1]:.01]
        grid = np.c_[xx.ravel(), yy.ravel()]
        probs = self.clf.predict_proba(grid)[:, 1].reshape(xx.shape)

        contour = self.ax.contourf(xx, yy, probs, 25, cmap="RdBu", vmin=0, vmax=1)
        # ax_c = self.figure.colorbar(contour)
        # ax_c.set_label("$P(y = 1)$")
        # ax_c.set_ticks([0, .25, .5, .75, 1])

        self.ax.scatter(self.X[:, 0], self.X[:, 1], c=self.Y, s=50,
                        cmap="RdBu", vmin=-.2, vmax=1.2,
                        edgecolor="white", linewidth=1)

        self.ax.set(aspect="auto",
                    xlim=(self.x_range[0], self.x_range[1]), ylim=(self.y_range[0], self.y_range[1]),
                    xlabel="$X_1$", ylabel="$X_2$")

        self.figure.canvas.draw()
        self.figure.canvas.flush_events()
        plt.show()

    def run(self, result):
        """ """
        # update new label and training the classifier based one the data
        self.update_guess(result)

        self._figure_update()

        a = self.update_data_x()

        # output the new point guess
        return a
