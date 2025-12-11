declare module '*.module.css' {
  const classes: { [key: string]: string };
  export default classes;
}

declare module '@theme/Layout' {
  import type * as React from 'react';

  export interface LayoutProps {
    readonly children?: React.ReactNode;
    readonly title?: string;
    readonly description?: string;
  }

  const Layout: React.ComponentType<LayoutProps>;
  export default Layout;
}


