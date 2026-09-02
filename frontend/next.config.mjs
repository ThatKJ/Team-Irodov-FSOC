/** @type {import('next').NextConfig} */
const nextConfig = {
  reactStrictMode: true,
  // three.js / R3F ship ESM that Next transpiles fine, but pin the transpile list
  // so drei helpers resolve in the app router server build.
  transpilePackages: ["three", "@react-three/fiber", "@react-three/drei"],
  eslint: {
    // lint is run explicitly via `npm run lint`; don't fail `next build` on style.
    ignoreDuringBuilds: false,
  },
};

export default nextConfig;
