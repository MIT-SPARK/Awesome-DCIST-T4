import {lazy, Suspense} from "react";
import {createHashRouter, Navigate, RouterProvider} from "react-router";
import {Shell} from "./components/Shell";
import {Loading} from "./components/ui";
import {MapsPage} from "./pages/Maps";

const ViewerPage = lazy(() => import("./viewer/ViewerPage"));

const router = createHashRouter([
  {
    path: "/",
    element: <Shell />,
    children: [
      {index: true, element: <Navigate to="/maps" replace />},
      {path: "maps", element: <MapsPage />},
      {
        path: "viewer",
        element: (
          <Suspense fallback={<Loading label="Loading 3D viewer" />}>
            <ViewerPage />
          </Suspense>
        ),
      },
      {path: "*", element: <Navigate to="/maps" replace />},
    ],
  },
]);

export function App() {
  return <RouterProvider router={router} />;
}
